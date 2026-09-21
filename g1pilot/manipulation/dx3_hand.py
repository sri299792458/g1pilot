#!/usr/bin/env python3

import copy
import threading
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_


DEX3_NUM_MOTORS = 7
DEX3_JOINT_ORDER = (
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "index_0",
    "index_1",
    "middle_0",
    "middle_1",
)

LEFT_JOINT_NAMES = tuple(f"left_hand_{name}_joint" for name in DEX3_JOINT_ORDER)
RIGHT_JOINT_NAMES = tuple(f"right_hand_{name}_joint" for name in DEX3_JOINT_ORDER)

LEFT_MAX_LIMITS = (1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0)
LEFT_MIN_LIMITS = (-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75)
RIGHT_MAX_LIMITS = (1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75)
RIGHT_MIN_LIMITS = (-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0)

DEFAULT_KP = 1.5
DEFAULT_KD = 0.1
MAX_DELTA_Q = 0.25

VALID_ARM_CONTROL_MODES = ("left", "right", "both")


def normalize_arm_controlled(arm_controlled):
    arm_controlled = arm_controlled.strip().lower()
    if arm_controlled not in VALID_ARM_CONTROL_MODES:
        raise ValueError(
            "arm_controlled must be one of: left, right, both "
            f"(got {arm_controlled!r})"
        )
    return arm_controlled


def make_mode(motor_id, status, timeout):
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clip_to_limit_ratio(desired_q, min_limit, max_limit, max_close_ratio):
    desired_q = clamp(desired_q, min_limit, max_limit)
    if desired_q > 0.0 and max_limit > 0.0:
        return min(desired_q, max_close_ratio * max_limit)
    if desired_q < 0.0 and min_limit < 0.0:
        return max(desired_q, max_close_ratio * min_limit)
    return desired_q


def size_hand_command(cmd):
    if len(cmd.motor_cmd) != DEX3_NUM_MOTORS:
        raise RuntimeError(
            f"Unitree HandCmd_ motor_cmd has length {len(cmd.motor_cmd)}, "
            f"expected {DEX3_NUM_MOTORS}."
        )


def make_default_command(kp=DEFAULT_KP, kd=DEFAULT_KD):
    cmd = unitree_hg_msg_dds__HandCmd_()
    size_hand_command(cmd)
    for i in range(DEX3_NUM_MOTORS):
        motor_cmd = cmd.motor_cmd[i]
        motor_cmd.mode = make_mode(i, status=0x01, timeout=0)
        motor_cmd.q = 0.0
        motor_cmd.dq = 0.0
        motor_cmd.kp = float(kp)
        motor_cmd.kd = float(kd)
        motor_cmd.tau = 0.0
    return cmd


def make_stop_command():
    cmd = unitree_hg_msg_dds__HandCmd_()
    size_hand_command(cmd)
    for i in range(DEX3_NUM_MOTORS):
        motor_cmd = cmd.motor_cmd[i]
        motor_cmd.mode = make_mode(i, status=0x01, timeout=0x01)
        motor_cmd.q = 0.0
        motor_cmd.dq = 0.0
        motor_cmd.kp = 0.0
        motor_cmd.kd = 0.0
        motor_cmd.tau = 0.0
    return cmd


def side_limits(side):
    if side == "left":
        return LEFT_MIN_LIMITS, LEFT_MAX_LIMITS
    return RIGHT_MIN_LIMITS, RIGHT_MAX_LIMITS


def side_joint_names(side):
    if side == "left":
        return LEFT_JOINT_NAMES
    return RIGHT_JOINT_NAMES


def close_pose(side):
    min_limits, max_limits = side_limits(side)
    return tuple((min_limits[i] + max_limits[i]) * 0.5 for i in range(DEX3_NUM_MOTORS))


@dataclass
class HandContext:
    side: str
    publisher: object = None
    subscriber: object = None
    last_state: object = None
    command: object = None

    def __post_init__(self):
        self.command = make_default_command()
        self.lock = threading.Lock()


class DX3Controller(Node):
    def __init__(self):
        super().__init__("dx3_hand_controller")

        self.declare_parameter("interface", "")
        self.declare_parameter("domain_id", 0)
        self.declare_parameter("arm_controlled", "both")
        self.declare_parameter("enable_dds", True)
        self.declare_parameter("send_commands", True)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("max_close_ratio", 1.0)

        self.interface = self.get_parameter("interface").get_parameter_value().string_value
        self.domain_id = int(self.get_parameter("domain_id").value)
        self.arm_controlled = normalize_arm_controlled(
            self.get_parameter("arm_controlled").get_parameter_value().string_value
        )
        self.enable_dds = bool(self.get_parameter("enable_dds").value)
        self.send_commands = bool(self.get_parameter("send_commands").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.max_close_ratio = clamp(float(self.get_parameter("max_close_ratio").value), 0.2, 1.0)

        if self.publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive")
        if self.enable_dds and not self.interface:
            raise RuntimeError("interface is required when enable_dds:=true")

        self.left = HandContext("left")
        self.right = HandContext("right")
        self._logged_first_state = {"left": False, "right": False}

        self.state_publishers = {
            "left": self.create_publisher(JointState, "/g1pilot/dx3/left/joint_states", 10),
            "right": self.create_publisher(JointState, "/g1pilot/dx3/right/joint_states", 10),
        }

        if self.arm_controlled in ("left", "both"):
            self.create_subscription(JointState, "/g1pilot/dx3/left/joint_target", self.left_target_callback, 1)
            self.create_subscription(String, "/g1pilot/dx3/left/command", self.left_command_callback, 1)
        if self.arm_controlled in ("right", "both"):
            self.create_subscription(JointState, "/g1pilot/dx3/right/joint_target", self.right_target_callback, 1)
            self.create_subscription(String, "/g1pilot/dx3/right/command", self.right_command_callback, 1)

        if self.enable_dds:
            self.get_logger().info(
                "Initializing Dex3 DDS: "
                f"domain_id={self.domain_id}, interface={self.interface!r}, "
                f"send_commands={self.send_commands}"
            )
            ChannelFactoryInitialize(self.domain_id, self.interface)
            self._initialize_hand_interfaces()
        else:
            self.get_logger().info("enable_dds:=false -> not connecting to Dex3 DDS.")

        self.create_timer(1.0 / self.publish_rate_hz, self.publish_once)

    def _initialize_hand_interfaces(self):
        if self.arm_controlled in ("left", "both"):
            if self.send_commands:
                self.left.publisher = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
                self.left.publisher.Init()
            self.left.subscriber = ChannelSubscriber("rt/dex3/left/state", HandState_)
            self.left.subscriber.Init(lambda msg: self._state_callback(self.left, msg), 1)

        if self.arm_controlled in ("right", "both"):
            if self.send_commands:
                self.right.publisher = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
                self.right.publisher.Init()
            self.right.subscriber = ChannelSubscriber("rt/dex3/right/state", HandState_)
            self.right.subscriber.Init(lambda msg: self._state_callback(self.right, msg), 1)

    def _state_callback(self, ctx, msg):
        with ctx.lock:
            ctx.last_state = copy.deepcopy(msg)
        if not self._logged_first_state[ctx.side]:
            self._logged_first_state[ctx.side] = True
            q0 = float(msg.motor_state[0].q)
            self.get_logger().info(f"Received first Dex3 {ctx.side} state from DDS; q0={q0:.4f}")
        self._publish_ros_state(ctx.side, msg)

    def _publish_ros_state(self, side, msg):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(side_joint_names(side))
        joint_state.position = [float(msg.motor_state[i].q) for i in range(DEX3_NUM_MOTORS)]
        joint_state.velocity = [float(msg.motor_state[i].dq) for i in range(DEX3_NUM_MOTORS)]
        self.state_publishers[side].publish(joint_state)

    def left_target_callback(self, msg):
        self._set_joint_target(self.left, msg)

    def right_target_callback(self, msg):
        self._set_joint_target(self.right, msg)

    def left_command_callback(self, msg):
        self._apply_named_command(self.left, msg.data)

    def right_command_callback(self, msg):
        self._apply_named_command(self.right, msg.data)

    def _set_joint_target(self, ctx, msg):
        if msg.name:
            name_to_position = dict(zip(msg.name, msg.position))
            try:
                q = [float(name_to_position[name]) for name in side_joint_names(ctx.side)]
            except KeyError as exc:
                self.get_logger().warn(f"{ctx.side} Dex3 target missing joint {exc.args[0]!r}")
                return
        else:
            if len(msg.position) != DEX3_NUM_MOTORS:
                self.get_logger().warn(
                    f"{ctx.side} Dex3 target has {len(msg.position)} positions; "
                    f"expected {DEX3_NUM_MOTORS}."
                )
                return
            q = [float(value) for value in msg.position]

        dq = [0.0] * DEX3_NUM_MOTORS
        if len(msg.velocity) == DEX3_NUM_MOTORS:
            dq = [float(value) for value in msg.velocity]

        kp = [DEFAULT_KP] * DEX3_NUM_MOTORS
        kd = [DEFAULT_KD] * DEX3_NUM_MOTORS
        self._set_all_joints_command(ctx, q, dq, kp, kd)

    def _apply_named_command(self, ctx, command):
        command = command.strip().lower().replace("-", "_")
        if command == "open":
            self._set_all_joints_command(ctx, [0.0] * DEX3_NUM_MOTORS)
        elif command == "close":
            self._set_all_joints_command(ctx, close_pose(ctx.side))
        elif command == "hold":
            with ctx.lock:
                state = copy.deepcopy(ctx.last_state)
            if state is None:
                self.get_logger().warn(f"Cannot hold {ctx.side} Dex3 hand before state is received.")
                return
            q = [float(state.motor_state[i].q) for i in range(DEX3_NUM_MOTORS)]
            self._set_all_joints_command(ctx, q)
        elif command == "stop":
            with ctx.lock:
                ctx.command = make_stop_command()
        else:
            self.get_logger().warn(
                f"Unknown {ctx.side} Dex3 command {command!r}; expected open, close, hold, or stop."
            )

    def _set_all_joints_command(self, ctx, q, dq=None, kp=None, kd=None, tau=None):
        if len(q) != DEX3_NUM_MOTORS:
            self.get_logger().warn(f"{ctx.side} Dex3 command has {len(q)} joints; expected {DEX3_NUM_MOTORS}.")
            return
        dq = [0.0] * DEX3_NUM_MOTORS if dq is None else list(dq)
        kp = [DEFAULT_KP] * DEX3_NUM_MOTORS if kp is None else list(kp)
        kd = [DEFAULT_KD] * DEX3_NUM_MOTORS if kd is None else list(kd)
        tau = [0.0] * DEX3_NUM_MOTORS if tau is None else list(tau)

        cmd = make_default_command()
        for i in range(DEX3_NUM_MOTORS):
            motor_cmd = cmd.motor_cmd[i]
            motor_cmd.mode = make_mode(i, status=0x01, timeout=0)
            motor_cmd.q = float(q[i])
            motor_cmd.dq = float(dq[i])
            motor_cmd.kp = float(kp[i])
            motor_cmd.kd = float(kd[i])
            motor_cmd.tau = float(tau[i])

        with ctx.lock:
            ctx.command = cmd

    def _smoothed_command(self, ctx):
        with ctx.lock:
            cmd = copy.deepcopy(ctx.command)
            state = copy.deepcopy(ctx.last_state)

        min_limits, max_limits = side_limits(ctx.side)
        for i in range(DEX3_NUM_MOTORS):
            motor_cmd = cmd.motor_cmd[i]
            desired_q = clip_to_limit_ratio(
                float(motor_cmd.q),
                min_limits[i],
                max_limits[i],
                self.max_close_ratio,
            )
            if state is not None and len(state.motor_state) == DEX3_NUM_MOTORS:
                current_q = float(state.motor_state[i].q)
                desired_q = current_q + clamp(desired_q - current_q, -MAX_DELTA_Q, MAX_DELTA_Q)
            motor_cmd.q = desired_q
        return cmd

    def publish_once(self):
        if not self.enable_dds or not self.send_commands:
            return
        if self.left.publisher is not None:
            self.left.publisher.Write(self._smoothed_command(self.left))
        if self.right.publisher is not None:
            self.right.publisher.Write(self._smoothed_command(self.right))


def main(args=None):
    rclpy.init(args=args)
    node = DX3Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
