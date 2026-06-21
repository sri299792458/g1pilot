#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from astroviz_interfaces.msg import MotorState, MotorStateList

#CLOSE_RIGHT_VALUES = [-0.10, 0.63, -1.74, 1.06, 0.95, 0.91, 1.22]
CLOSE_RIGHT_VALUES_1 = [0.00,  0.0,  0.0, 1.2, 1.6, 0.0, 0.0] # 1 finger
CLOSE_RIGHT_VALUES_2 = [0.00,  0.0,  0.0, 1.2, 1.6, 1.2, 1.6] # closed Hand
CLOSE_LEFT_VALUES_1  = [0.04,  0.6,  1.4, -1.2, -1.4, -0.0, 0.0] # 1 finger
CLOSE_LEFT_VALUES_2  = [0.04,  0.6,  1.4, -1.2, -1.6, -1.2, -1.4] # closed Hand
# CLOSE_LEFT_VALUES  = [0.04,  -0.04,  1.51, -1.10, -1.47, -1.13, -1.23]
# CLOSE_LEFT_VALUES  = [0.04,  0.4,  1.5, -1.10, -1.58, -1.13, -1.32] motor gripper

OPEN_VALUES          = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

VALID_ARM_CONTROL_MODES = ("left", "right", "both")
RIGHT_DX3_ACTION_TOPIC = "/g1pilot/dx3/hand_action/right"
LEFT_DX3_ACTION_TOPIC = "/g1pilot/dx3/hand_action/left"
RIGHT_DX3_MOTOR_STATE_TOPIC = "/g1pilot/dx3/right/motor_state"
LEFT_DX3_MOTOR_STATE_TOPIC = "/g1pilot/dx3/left/motor_state"

def normalize_arm_controlled(arm_controlled):
    arm_controlled = arm_controlled.strip().lower()
    if arm_controlled not in VALID_ARM_CONTROL_MODES:
        raise ValueError(
            "arm_controlled must be one of: left, right, both "
            f"(got {arm_controlled!r})"
        )
    return arm_controlled

def normalize_hand_action(action):
    action = action.strip().lower().replace("-", "_")
    if action not in ("open", "pinch", "close"):
        raise ValueError(
            "DX3 hand action must be one of: open, pinch, close "
            f"(got {action!r})"
        )
    return action

def dx3_target(side, action):
    if action == "open":
        return OPEN_VALUES
    if side == "right":
        return CLOSE_RIGHT_VALUES_1 if action == "pinch" else CLOSE_RIGHT_VALUES_2
    return CLOSE_LEFT_VALUES_1 if action == "pinch" else CLOSE_LEFT_VALUES_2

class DX3Controller(Node):
    def __init__(self):
        super().__init__('dx3_hand_controller')
        self.declare_parameter("interface", "")
        self.declare_parameter("arm_controlled", "both")
        self.declare_parameter("use_robot", True)
        self.declare_parameter("send_commands", True)
        interface = self.get_parameter("interface").get_parameter_value().string_value
        arm_controlled = normalize_arm_controlled(
            self.get_parameter("arm_controlled").get_parameter_value().string_value
        )
        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.send_commands = bool(self.get_parameter("send_commands").value) and self.use_robot
        self.left_gripper_state_publisher = self.create_publisher(
            MotorStateList, LEFT_DX3_MOTOR_STATE_TOPIC, QoSProfile(depth=10)
        )
        self.right_gripper_state_publisher = self.create_publisher(
            MotorStateList, RIGHT_DX3_MOTOR_STATE_TOPIC, QoSProfile(depth=10)
        )

        self.right_action = None
        self.left_action = None
        self.right_target = OPEN_VALUES
        self.left_target = OPEN_VALUES
        self.total_motors = 7

        if self.use_robot and not interface:
            raise RuntimeError("interface is required when use_robot:=true")

        if arm_controlled in ["right", "both"]:
            self.create_subscription(String, RIGHT_DX3_ACTION_TOPIC, self.right_action_callback, 10)

        if arm_controlled in ["left", "both"]:
            self.create_subscription(String, LEFT_DX3_ACTION_TOPIC, self.left_action_callback, 10)

        if self.use_robot:
            self.get_logger().info("use_robot:=true -> Initializing Unitree DX3 hand DDS interface")
            ChannelFactoryInitialize(0, interface)
            self.initialize_hand_interfaces(arm_controlled)
        else:
            self.get_logger().info("use_robot:=false -> Not connecting to Unitree DX3 hand DDS interface.")

        self.create_timer(0.05, self.publish_commands)

    def initialize_hand_interfaces(self, arm_controlled):
        if arm_controlled in ["right", "both"]:
            if self.send_commands:
                self.right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
                self.right_pub.Init()
            self.right_sub = ChannelSubscriber("rt/dex3/right/state", HandState_)
            self.right_sub.Init(self.right_callback)

        if arm_controlled in ["left", "both"]:
            if self.send_commands:
                self.left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
                self.left_pub.Init()
            self.left_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
            self.left_sub.Init(self.left_callback)

    def right_action_callback(self, msg: String):
        self._set_hand_action("right", msg.data)

    def left_action_callback(self, msg: String):
        self._set_hand_action("left", msg.data)

    def _set_hand_action(self, side, action):
        try:
            normalized_action = normalize_hand_action(action)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        if side == "right":
            self.right_action = normalized_action
            self.right_target = dx3_target(side, normalized_action)
        else:
            self.left_action = normalized_action
            self.left_target = dx3_target(side, normalized_action)

        self.get_logger().info(f"DX3 {side} hand action: {normalized_action}")

    def left_callback(self, msg: HandState_):
        motor_list_msg = MotorStateList()
        positions = []
        for i in range(len(msg.motor_state)):
            motor_state = MotorState()
            motor_state.name = f"left_motor_{i}"
            motor_state.temperature = float(msg.motor_state[i].temperature[0])
            motor_state.voltage = float(msg.motor_state[i].vol)
            motor_state.position = float(msg.motor_state[i].q)
            motor_state.velocity = float(msg.motor_state[i].dq)
            motor_list_msg.motor_list.append(motor_state)
            positions.append(motor_state.position)
        self.left_gripper_state_publisher.publish(motor_list_msg)

        if  self.send_commands:
            return
        self.get_logger().debug(f'Left hand positions: {positions}')

    def right_callback(self, msg: HandState_):
        motor_list_msg = MotorStateList()
        positions = []
        for i in range(len(msg.motor_state)):
            motor_state = MotorState()
            motor_state.name = f"right_motor_{i}"
            motor_state.temperature = float(msg.motor_state[i].temperature[0])
            motor_state.voltage = float(msg.motor_state[i].vol)
            motor_state.position = float(msg.motor_state[i].q)
            motor_state.velocity = float(msg.motor_state[i].dq)
            motor_list_msg.motor_list.append(motor_state)
            positions.append(motor_state.position)
        self.right_gripper_state_publisher.publish(motor_list_msg)

        if  self.send_commands:
            return
        self.get_logger().debug(f'Right hand positions: {positions}')

    def create_cmd(self, values):
        cmd = unitree_hg_msg_dds__HandCmd_()
        for i in range(self.total_motors):
            cmd.motor_cmd[i].mode = 0
            cmd.motor_cmd[i].q = values[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 1.5
            cmd.motor_cmd[i].kd = 0.2
        return cmd

    def publish_commands(self):
        if not self.send_commands:
            return
        if hasattr(self, "right_pub") and self.right_action is not None:
            self.right_pub.Write(self.create_cmd(self.right_target))
        if hasattr(self, "left_pub") and self.left_action is not None:
            self.left_pub.Write(self.create_cmd(self.left_target))

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

if __name__ == '__main__':
    main()
