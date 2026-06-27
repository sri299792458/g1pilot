#!/usr/bin/env python3

import collections

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.simulation.lowcmd_utils import (
    G1_NUM_MOTORS,
    LEG_MOTOR_IDS,
    OPENHOMIE_OBS_MOTOR_IDS,
    clamp,
    copy_command_metadata,
    motor_dq,
    motor_q,
    parse_int_list,
    set_motor_command,
    set_passive_motor_command,
)
from g1pilot.simulation.openhomie_policy import (
    OpenHomiePolicy,
    compute_openhomie_observation,
    openhomie_single_obs_dim,
)


DEFAULT_KPS = [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0]
DEFAULT_KDS = [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0]
DEFAULT_ANGLES = [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]
DEFAULT_CMD_SCALE = [2.0, 2.0, 0.25]


class OpenHomieLowCmdBase(Node):
    def __init__(self):
        super().__init__("openhomie_lowcmd_base")

        self.declare_parameter("domain_id", 1)
        self.declare_parameter("interface", "lo")
        self.declare_parameter("lowstate_topic", "rt/lowstate")
        self.declare_parameter("base_cmd_topic", "rt/lowcmd_base")
        self.declare_parameter("cmd_vel_topic", "/g1pilot/sim_loco/cmd_vel")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("allow_zero_policy", False)
        self.declare_parameter("mode", "stand")
        self.declare_parameter("control_decimation", 10)
        self.declare_parameter("obs_history_len", 6)
        self.declare_parameter("expected_num_obs", 456)
        self.declare_parameter("height_cmd", 0.34)
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("ang_vel_scale", 0.25)
        self.declare_parameter("dof_pos_scale", 1.0)
        self.declare_parameter("dof_vel_scale", 0.05)
        self.declare_parameter("max_vx", 0.3)
        self.declare_parameter("max_vy", 0.2)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("leg_motor_ids", ",".join(str(i) for i in LEG_MOTOR_IDS))
        self.declare_parameter("obs_motor_ids", ",".join(str(i) for i in OPENHOMIE_OBS_MOTOR_IDS))
        self.declare_parameter("kps", DEFAULT_KPS)
        self.declare_parameter("kds", DEFAULT_KDS)
        self.declare_parameter("default_angles", DEFAULT_ANGLES)
        self.declare_parameter("cmd_scale", DEFAULT_CMD_SCALE)

        self.domain_id = int(self.get_parameter("domain_id").value)
        self.interface = self.get_parameter("interface").get_parameter_value().string_value
        self.lowstate_topic = self.get_parameter("lowstate_topic").get_parameter_value().string_value
        self.base_cmd_topic = self.get_parameter("base_cmd_topic").get_parameter_value().string_value
        self.policy_path = self.get_parameter("policy_path").get_parameter_value().string_value
        self.allow_zero_policy = bool(self.get_parameter("allow_zero_policy").value)
        self.mode = self.get_parameter("mode").get_parameter_value().string_value.strip().lower()
        self.control_decimation = int(self.get_parameter("control_decimation").value)
        self.obs_history_len = int(self.get_parameter("obs_history_len").value)
        self.expected_num_obs = int(self.get_parameter("expected_num_obs").value)
        self.height_cmd = float(self.get_parameter("height_cmd").value)
        self.action_scale = float(self.get_parameter("action_scale").value)
        self.ang_vel_scale = float(self.get_parameter("ang_vel_scale").value)
        self.dof_pos_scale = float(self.get_parameter("dof_pos_scale").value)
        self.dof_vel_scale = float(self.get_parameter("dof_vel_scale").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.leg_motor_ids = parse_int_list(self.get_parameter("leg_motor_ids").value, LEG_MOTOR_IDS)
        self.obs_motor_ids = parse_int_list(self.get_parameter("obs_motor_ids").value, OPENHOMIE_OBS_MOTOR_IDS)
        self.kps = np.asarray(self.get_parameter("kps").value, dtype=np.float32)
        self.kds = np.asarray(self.get_parameter("kds").value, dtype=np.float32)
        self.default_angles = np.asarray(self.get_parameter("default_angles").value, dtype=np.float32)
        self.cmd_scale = np.asarray(self.get_parameter("cmd_scale").value, dtype=np.float32)

        self._validate_parameters()

        self.single_obs_dim = openhomie_single_obs_dim(len(self.obs_motor_ids), len(self.leg_motor_ids))
        self.num_obs = self.single_obs_dim * self.obs_history_len
        if self.num_obs != self.expected_num_obs:
            raise RuntimeError(
                f"OpenHomie observation size mismatch: got {self.num_obs}, expected {self.expected_num_obs}. "
                "Check obs_motor_ids, leg_motor_ids, and obs_history_len."
            )

        self.policy = self._load_policy()
        self.last_action = np.zeros(len(self.leg_motor_ids), dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()
        self.command = np.zeros(3, dtype=np.float32)
        self.obs_history = collections.deque(maxlen=self.obs_history_len)
        for _ in range(self.obs_history_len):
            self.obs_history.append(np.zeros(self.single_obs_dim, dtype=np.float32))

        self.last_policy_tick = None
        self.physics_sample_count = 0
        self._warned_static_tick = False
        self._reported_lowstate = False
        self._reported_publish = False
        self._reported_policy_start = False
        self.crc = CRC()

        self.get_logger().info(
            f"Initializing Unitree DDS for OpenHomie baseline on interface={self.interface!r}, "
            f"domain_id={self.domain_id}, output={self.base_cmd_topic!r}"
        )
        ChannelFactoryInitialize(self.domain_id, self.interface)
        self.lowstate_subscriber = ChannelSubscriber(self.lowstate_topic, LowState_)
        self.lowstate_subscriber.Init(self._lowstate_callback, 1)
        self.base_cmd_publisher = ChannelPublisher(self.base_cmd_topic, LowCmd_)
        self.base_cmd_publisher.Init()

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.get_logger().info(
            "OpenHomie base commands are driven by rt/lowstate callbacks; "
            f"policy updates every {self.control_decimation} fresh LowState samples."
        )

    def _validate_parameters(self):
        if self.mode not in ("stand", "walk"):
            raise ValueError(f"mode must be 'stand' or 'walk', got {self.mode!r}")
        if self.control_decimation <= 0:
            raise ValueError("control_decimation must be positive")
        if len(self.leg_motor_ids) != len(DEFAULT_ANGLES):
            raise ValueError("leg_motor_ids must contain 12 motor IDs for the OpenHomie policy")
        if len(self.kps) != len(self.leg_motor_ids):
            raise ValueError("kps length must match leg_motor_ids length")
        if len(self.kds) != len(self.leg_motor_ids):
            raise ValueError("kds length must match leg_motor_ids length")
        if len(self.default_angles) != len(self.leg_motor_ids):
            raise ValueError("default_angles length must match leg_motor_ids length")
        if len(self.cmd_scale) != 3:
            raise ValueError("cmd_scale must have length 3")

    def _load_policy(self):
        if not self.policy_path:
            if self.allow_zero_policy:
                self.get_logger().warn("policy_path is empty; using zero-action policy for smoke testing.")
                return None
            raise RuntimeError("policy_path is required unless allow_zero_policy:=true")
        policy = OpenHomiePolicy(self.policy_path, num_obs=self.num_obs, num_actions=len(self.leg_motor_ids))
        self.get_logger().info(
            f"Loaded OpenHomie policy: {self.policy_path} "
            f"({policy.backend}, {policy.input_shape} -> {policy.output_shape})"
        )
        return policy

    def _lowstate_callback(self, msg):
        if not self._reported_lowstate:
            self.get_logger().info(
                f"Received first simulated LowState on {self.lowstate_topic!r}; "
                f"mode_machine={getattr(msg, 'mode_machine', 0)}, tick={getattr(msg, 'tick', 0)}"
            )
            self._reported_lowstate = True
        self._process_lowstate(msg)

    def _cmd_vel_callback(self, msg):
        self.command[0] = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self.command[1] = clamp(msg.linear.y, -self.max_vy, self.max_vy)
        self.command[2] = clamp(msg.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

    def _active_command(self):
        if self.mode == "stand":
            return np.zeros(3, dtype=np.float32)
        return self.command.copy()

    def _compute_observation(self, lowstate):
        qj = np.array([motor_q(lowstate, motor_id) for motor_id in self.obs_motor_ids], dtype=np.float32)
        dqj = np.array([motor_dq(lowstate, motor_id) for motor_id in self.obs_motor_ids], dtype=np.float32)
        quat = np.asarray(lowstate.imu_state.quaternion, dtype=np.float32)
        omega = np.asarray(lowstate.imu_state.gyroscope, dtype=np.float32)
        return compute_openhomie_observation(
            command=self._active_command(),
            height_cmd=self.height_cmd,
            omega=omega,
            quat_wxyz=quat,
            qj=qj,
            dqj=dqj,
            last_action=self.last_action,
            default_angles=self.default_angles,
            cmd_scale=self.cmd_scale,
            ang_vel_scale=self.ang_vel_scale,
            dof_pos_scale=self.dof_pos_scale,
            dof_vel_scale=self.dof_vel_scale,
        )

    def _run_policy(self, obs):
        if self.policy is None:
            return np.zeros(len(self.leg_motor_ids), dtype=np.float32)
        return self.policy.run(obs)

    def _policy_update_due(self, lowstate):
        current_tick = int(getattr(lowstate, "tick", 0))
        if self.last_policy_tick is None:
            self.last_policy_tick = current_tick
            return False

        if current_tick <= self.last_policy_tick:
            if not self._warned_static_tick:
                self.get_logger().warn(
                    "LowState tick has not advanced; publishing current target without advancing policy."
                )
                self._warned_static_tick = True
            return False

        self.last_policy_tick = current_tick
        self.physics_sample_count += 1
        self._warned_static_tick = False
        return self.physics_sample_count % self.control_decimation == 0

    def _build_lowcmd_base(self, lowstate):
        cmd = unitree_hg_msg_dds__LowCmd_()
        copy_command_metadata(cmd, lowstate=lowstate)
        cmd.mode_pr = 0

        for motor_id in range(G1_NUM_MOTORS):
            set_passive_motor_command(cmd, lowstate, motor_id)

        for i, motor_id in enumerate(self.leg_motor_ids):
            set_motor_command(
                cmd,
                motor_id,
                mode=1,
                q=float(self.target_dof_pos[i]),
                kp=float(self.kps[i]),
                kd=float(self.kds[i]),
            )

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def _process_lowstate(self, lowstate):
        base_cmd = self._build_lowcmd_base(lowstate)
        self.base_cmd_publisher.Write(base_cmd)
        if not self._reported_publish:
            targets = ", ".join(f"{value:.3f}" for value in self.target_dof_pos)
            self.get_logger().info(f"Published first OpenHomie base command to {self.base_cmd_topic!r}: [{targets}]")
            self._reported_publish = True

        if not self._policy_update_due(lowstate):
            return

        single_obs = self._compute_observation(lowstate)
        self.obs_history.append(single_obs)

        obs = np.concatenate(tuple(self.obs_history)).astype(np.float32)
        self.last_action = self._run_policy(obs)
        self.target_dof_pos = self.last_action * self.action_scale + self.default_angles
        self.base_cmd_publisher.Write(self._build_lowcmd_base(lowstate))
        if not self._reported_policy_start:
            action_text = ", ".join(f"{value:.4f}" for value in self.last_action)
            target_text = ", ".join(f"{value:.4f}" for value in self.target_dof_pos)
            self.get_logger().info(
                f"Started OpenHomie policy updates at LowState tick={getattr(lowstate, 'tick', 0)} "
                f"after {self.physics_sample_count} fresh LowState samples "
                f"with control_decimation={self.control_decimation}"
            )
            self.get_logger().info(f"First OpenHomie policy action: [{action_text}]")
            self.get_logger().info(f"First OpenHomie policy target: [{target_text}]")
            self._reported_policy_start = True


def main(args=None):
    rclpy.init(args=args)
    node = OpenHomieLowCmdBase()
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
