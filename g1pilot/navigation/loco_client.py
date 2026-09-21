#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from sensor_msgs.msg import Joy

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

EXPECTED_JOY_AXES = 5
EXPECTED_JOY_BUTTONS = 8


def _rpc_get_int(client, api_id):
    try:
        code, data = client._Call(api_id, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


class G1LocoClient(Node):
    def __init__(self):
        super().__init__("loco_client")
        self.robot_stopped = False
        self.balanced = False
        self.prev_buttons = {}
        self.prev_axis_last = None
        self.control_arms = False

        self.declare_parameter('use_robot', True)
        self.use_robot = bool(self.get_parameter('use_robot').value)

        self.declare_parameter('interface', '')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.declare_parameter('arm_controlled', 'both')
        self.arm_controlled = self.get_parameter('arm_controlled').get_parameter_value().string_value
        self.declare_parameter('enable_arm_ui', True)
        self.enable_arm_ui = self.get_parameter('enable_arm_ui').get_parameter_value().bool_value

        self.create_subscription(
            Float64,
            '/base_height',
            self.base_height_callback,
            10
        )

        if self.use_robot:
            ChannelFactoryInitialize(0, interface)
            self.robot = LocoClient()
            self.robot.SetTimeout(10.0)
            self.robot.Init()
            self.robot.SetFsmId(4)
            self.robot.Damp()
            self.current_id = self.get_fsm_id()
            self.current_mode = self.get_fsm_mode()
            self.get_logger().info(f"Current FSM ID: {self.current_id}, Mode: {self.current_mode}")
        else:
            self.robot = None
            self.current_id = 4
            self.current_mode = 0
            self.get_logger().info("use_robot:=false -> Not connecting to robot.")

        self.create_subscription(Bool, '/g1pilot/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Bool, '/g1pilot/start', self.start_callback, 10)
        self.create_subscription(Bool, '/g1pilot/start_balancing', self.start_balancing_callback, 10)
        self.create_subscription(Joy, '/g1pilot/joy', self.joystick_callback, 10)

        self.publisher_arms_controlled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 1)
        self.right_gripper_pub = self.create_publisher(String, '/g1pilot/dx3/hand_action/right', 1)
        self.left_gripper_pub = self.create_publisher(String, '/g1pilot/dx3/hand_action/left', 1)
        self.publisher_homming_arms = self.create_publisher(Bool, '/g1pilot/arms/home', 1)

    def _log_once(self, level, msg, key):
        if getattr(self, key, False):
            return
        setattr(self, key, True)

        logger = self.get_logger()
        if level == "info":
            logger.info(msg)
        elif level == "warn":
            logger.warn(msg)
        elif level == "error":
            logger.error(msg)
        elif level == "debug":
            logger.debug(msg)
        else:
            logger.info(msg)


    def _clear_once(self, key):
        if hasattr(self, key):
            delattr(self, key)

    def _warn_if_short_joy(self, msg):
        missing = []
        if len(msg.axes) < EXPECTED_JOY_AXES:
            missing.append(f"axes={len(msg.axes)} < {EXPECTED_JOY_AXES}")
        if len(msg.buttons) < EXPECTED_JOY_BUTTONS:
            missing.append(f"buttons={len(msg.buttons)} < {EXPECTED_JOY_BUTTONS}")

        if missing:
            self._log_once(
                "warn",
                f"Ignoring missing joystick fields as neutral values ({', '.join(missing)}).",
                "_short_joy_logged",
            )
        else:
            self._clear_once("_short_joy_logged")

    def _axis(self, msg, idx, default=0.0):
        if 0 <= idx < len(msg.axes):
            return msg.axes[idx]
        return default

    def _button(self, msg, idx, default=0):
        if 0 <= idx < len(msg.buttons):
            return msg.buttons[idx]
        return default

    def _btn_rising(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return self._button(msg, idx) == 1 and prev == 0

    def _btn_falling(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return self._button(msg, idx) == 0 and prev == 1

    def _axis_edge(self, cur, prev, val):
        return cur == val and prev != val, cur != val and prev == val

    def get_fsm_id(self):
        if not self.use_robot or self.robot is None:
            return 4
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)

    def get_fsm_mode(self):
        if not self.use_robot or self.robot is None:
            return 0
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_MODE)

    def emergency_callback(self, msg: Bool):
        if msg.data:
            self._log_once("warn", "EMERGENCY STOP ACTIVATED!", "_e_stop_activated_logged")
            self.robot_stopped = True
            self.balanced = False
            if self.use_robot and self.robot is not None:
                self.robot.Damp()
            if self.control_arms:
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
        else:
            self._clear_once("_e_stop_activated_logged")

    def base_height_callback(self, msg: Float64):
        # self.get_logger().warning(f"Received base height command: {msg.data}")
        if self.use_robot and self.robot is not None:
            self.robot.SetStandHeight(msg.data)

    def start_callback(self, msg: Bool):
        if self.use_robot and self.robot is not None and msg.data:
            self.robot.SetFsmId(4)
            self._log_once("info", "Switched to FSM ID 4 (Standby)", "_switch_fsm_id_4_logged")
            self.robot_stopped = False
            self.balanced = False

    def start_balancing_callback(self, msg: Bool):
        if msg.data and not self.balanced:
            self._log_once("info", "Starting balancing procedure...", "_start_balance_req_logged")
            if self.entering_balancing(max_height=0.5, step=0.02):
                self._log_once("info", "Balancing procedure completed.", "_balance_completed_logged")
            else:
                self._log_once("warn", "Balancing procedure did not complete.", "_balance_failed_logged")
        elif self.balanced:
            self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_logged")

    def joystick_callback(self, msg: Joy):
        try:
            self._warn_if_short_joy(msg)

            if not self.prev_buttons:
                self.prev_buttons = {
                    i: 0 for i in range(max(len(msg.buttons), EXPECTED_JOY_BUTTONS))
                }

            if not self.balanced:
                self._log_once("warn", "Robot is not balanced, cannot move.", "_warn_not_balanced_logged")
            else:
                self._clear_once("_warn_not_balanced_logged")

            if self.robot_stopped:
                self._log_once("warn", "Robot is stopped, cannot move.", "_warn_robot_stopped_logged")
            else:
                self._clear_once("_warn_robot_stopped_logged")

            axis_last = msg.axes[-1] if len(msg.axes) else 0.0
            if self.prev_axis_last is None:
                self.prev_axis_last = axis_last

            up_on, up_off = self._axis_edge(axis_last, self.prev_axis_last, -1.0)
            if up_on:
                if self.use_robot and self.robot is not None:
                    self.robot.SetFsmId(4)
                self._log_once("info", "Switched to FSM ID 4 (Standby)", "_switch_fsm_id_4_logged")
                self.robot_stopped = False
                self.balanced = False
            if up_off:
                self._clear_once("_switch_fsm_id_4_logged")

            if self._btn_rising(msg, 0):
                self.control_arms = not self.control_arms
                if self.control_arms:
                    self._log_once("info", "Enabling arm control mode.", "_enable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=True))
                else:
                    self._log_once("info", "Disabling arm control mode.", "_disable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=False))

            if self._btn_rising(msg, 1):
                if self.control_arms:
                    self._log_once("info", "Moving arms to home position.", "_move_arms_home_logged")
                    self.publisher_homming_arms.publish(Bool(data=True))
                else:
                    self._log_once("warn", "Cannot move arms to home, arm control mode is disabled.", "_warn_move_home_no_control_logged")

            if self._btn_rising(msg, 5):
                self._log_once("warn", "Emergency stop button pressed!", "_e_stop_button_pressed_logged")
                self.robot_stopped = True
                self.balanced = False
                if self.use_robot and self.robot is not None:
                    self.robot.Damp()
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
            if self._btn_falling(msg, 5):
                self._clear_once("_e_stop_button_pressed_logged")

            # Gripper controls
            gripper_side_axis = self._axis(msg, 4)
            if gripper_side_axis == 1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open right gripper.", "_open_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="open"))
            if gripper_side_axis == 1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close right gripper.", "_close_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="close"))

            if gripper_side_axis == -1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open left gripper.", "_open_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="open"))
            if gripper_side_axis == -1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close left gripper.", "_close_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="close"))

            if self._btn_rising(msg, 4):
                if not self.balanced:
                    self._log_once("info", "Starting balancing procedure...", "_start_balance_r1_logged")
                    if self.entering_balancing(max_height=0.5, step=0.02):
                        self._log_once("info", "Balancing procedure completed.", "_balance_completed_r1_logged")
                    else:
                        self._log_once("warn", "Balancing procedure did not complete.", "_balance_failed_r1_logged")
                else:
                    self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_r1_logged")
            if self._btn_falling(msg, 4):
                self._clear_once("_start_balance_r1_logged")
                self._clear_once("_balance_completed_r1_logged")
                self._clear_once("_balance_failed_r1_logged")
                self._clear_once("_already_balanced_notice_r1_logged")

            if self._button(msg, 5) == 0 and not self.robot_stopped and self.balanced:
                if self.use_robot and self.robot is not None:
                    self.robot.StopMove()

            if self._button(msg, 7) == 1 and not self.robot_stopped and self.balanced:
                vx = round(self._axis(msg, 1) * -0.5, 2)
                vy = round(self._axis(msg, 0) * -0.5, 2)
                yaw = round(self._axis(msg, 2) * -0.5, 2)
                self._log_once("info", f"Moving with vx: {vx}, vy: {vy}, yaw: {yaw}", "_moving_logged")
                if self.use_robot and self.robot is not None:
                    if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(yaw) < 0.03:
                        self.robot.StopMove()
                    else:
                        self.robot.Move(vx=vx, vy=vy, vyaw=yaw, continous_move=True)

            self.prev_buttons = {
                i: self._button(msg, i)
                for i in range(max(len(msg.buttons), EXPECTED_JOY_BUTTONS))
            }
            self.prev_axis_last = axis_last

        except Exception as e:
            self.get_logger().error(f"Error in joystick_callback: {e}")
            if self.use_robot and self.robot is not None:
                self.robot.StopMove()
                self.robot.Damp()
            self.robot_stopped = True
            self.balanced = False

    def entering_balancing(self, max_height=0.5, step=0.02):
        if not self.use_robot or self.robot is None:
            self.balanced = True
            self.get_logger().info("Sim balancing done (use_robot:=false).")
            return True
        height = 0.0
        reached_start_height = False
        while height < max_height and not self.robot_stopped:
            height += step
            self.robot.SetStandHeight(height)
            fsm_mode = self.get_fsm_mode()
            if fsm_mode == 0 and height >= 0.2:
                self._log_once("info", f"Reached max height: {height}", "_balance_reach_height_logged")
                reached_start_height = True
                break
            elif fsm_mode != 0:
                self._log_once(
                    "warn",
                    f"Problems during balancing, stopping before BalanceStand/Start. FSM mode: {fsm_mode}",
                    "_balance_problem_stop_logged",
                )
                self.balanced = False
                return False

        if self.robot_stopped:
            self._log_once(
                "warn",
                "Balancing aborted because robot is stopped.",
                "_balance_aborted_robot_stopped_logged",
            )
            self.balanced = False
            return False

        if not reached_start_height:
            self._log_once(
                "warn",
                "Balancing did not reach the required start height.",
                "_balance_height_not_reached_logged",
            )
            self.balanced = False
            return False

        self.robot.BalanceStand(1)
        self.robot.SetStandHeight(height)
        self.robot.Start()
        self.balanced = True
        return True


def main(args=None):
    rclpy.init(args=args)
    node = G1LocoClient()
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
