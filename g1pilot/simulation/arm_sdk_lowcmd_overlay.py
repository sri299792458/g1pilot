#!/usr/bin/env python3

import time
from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.simulation.lowcmd_utils import (
    ARM_MOTOR_IDS,
    G1_NUM_MOTORS,
    LEG_MOTOR_IDS,
    LOCKED_WAIST_MOTOR_IDS,
    WAIST_YAW_MOTOR_ID,
    copy_command_metadata,
    copy_motor_command,
    motor_q,
    parse_int_list,
    parse_float_list,
    set_motor_command,
)


DEFAULT_OVERLAY_MOTOR_IDS = (WAIST_YAW_MOTOR_ID,) + ARM_MOTOR_IDS
DEFAULT_OVERLAY_NOMINAL_POSITIONS = (
    0.0,
    0.3,
    0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,
    0.3,
    -0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,
)
DEFAULT_LOCKED_POSITIONS = (0.0, 0.0)


class ArmSdkLowCmdOverlay(Node):
    def __init__(self):
        super().__init__("arm_sdk_lowcmd_overlay")

        self.declare_parameter("domain_id", 1)
        self.declare_parameter("interface", "lo")
        self.declare_parameter("base_cmd_topic", "rt/lowcmd_base")
        self.declare_parameter("arm_sdk_topic", "rt/arm_sdk")
        self.declare_parameter("lowstate_topic", "rt/lowstate")
        self.declare_parameter("final_cmd_topic", "rt/lowcmd")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("base_timeout_s", 0.25)
        self.declare_parameter("arm_timeout_s", 0.25)
        self.declare_parameter("leg_motor_ids", ",".join(str(i) for i in LEG_MOTOR_IDS))
        self.declare_parameter("overlay_motor_ids", ",".join(str(i) for i in DEFAULT_OVERLAY_MOTOR_IDS))
        self.declare_parameter("overlay_nominal_positions", list(DEFAULT_OVERLAY_NOMINAL_POSITIONS))
        self.declare_parameter("locked_motor_ids", ",".join(str(i) for i in LOCKED_WAIST_MOTOR_IDS))
        self.declare_parameter("locked_positions", list(DEFAULT_LOCKED_POSITIONS))
        self.declare_parameter("hold_kp", 20.0)
        self.declare_parameter("hold_kd", 2.0)

        self.domain_id = int(self.get_parameter("domain_id").value)
        self.interface = self.get_parameter("interface").get_parameter_value().string_value
        self.base_cmd_topic = self.get_parameter("base_cmd_topic").get_parameter_value().string_value
        self.arm_sdk_topic = self.get_parameter("arm_sdk_topic").get_parameter_value().string_value
        self.lowstate_topic = self.get_parameter("lowstate_topic").get_parameter_value().string_value
        self.final_cmd_topic = self.get_parameter("final_cmd_topic").get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.base_timeout_s = float(self.get_parameter("base_timeout_s").value)
        self.arm_timeout_s = float(self.get_parameter("arm_timeout_s").value)
        self.leg_motor_ids = parse_int_list(self.get_parameter("leg_motor_ids").value, LEG_MOTOR_IDS)
        self.overlay_motor_ids = parse_int_list(
            self.get_parameter("overlay_motor_ids").value, DEFAULT_OVERLAY_MOTOR_IDS
        )
        self.overlay_nominal_positions = parse_float_list(
            self.get_parameter("overlay_nominal_positions").value, DEFAULT_OVERLAY_NOMINAL_POSITIONS
        )
        self.locked_motor_ids = parse_int_list(self.get_parameter("locked_motor_ids").value, LOCKED_WAIST_MOTOR_IDS)
        self.locked_positions = parse_float_list(self.get_parameter("locked_positions").value, DEFAULT_LOCKED_POSITIONS)
        self.hold_kp = float(self.get_parameter("hold_kp").value)
        self.hold_kd = float(self.get_parameter("hold_kd").value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        for motor_id in self.locked_motor_ids:
            if motor_id in self.overlay_motor_ids:
                raise ValueError(f"locked motor {motor_id} cannot also be in overlay_motor_ids")
        if len(self.overlay_nominal_positions) != len(self.overlay_motor_ids):
            raise ValueError("overlay_nominal_positions length must match overlay_motor_ids length")
        if len(self.locked_positions) != len(self.locked_motor_ids):
            raise ValueError("locked_positions length must match locked_motor_ids length")
        if any(motor_id in self.overlay_motor_ids or motor_id in self.locked_motor_ids for motor_id in self.leg_motor_ids):
            raise ValueError("leg_motor_ids must not overlap overlay or locked motor IDs")

        self.base_cmd = None
        self.base_time = None
        self.arm_cmd = None
        self.arm_time = None
        self.lowstate = None
        self.lowstate_time = None
        self._warned_no_base = False
        self._warned_no_lowstate = False
        self._warned_base_timeout = False
        self._warned_arm_timeout = False
        self._reported_lowstate = False
        self._reported_base = False
        self._reported_final = False
        self.crc = CRC()

        self.get_logger().info(
            f"Initializing Unitree DDS overlay on interface={self.interface!r}, domain_id={self.domain_id}; "
            f"final writer={self.final_cmd_topic!r}"
        )
        ChannelFactoryInitialize(self.domain_id, self.interface)

        self.base_subscriber = ChannelSubscriber(self.base_cmd_topic, LowCmd_)
        self.base_subscriber.Init(self._base_callback, 1)
        self.arm_subscriber = ChannelSubscriber(self.arm_sdk_topic, LowCmd_)
        self.arm_subscriber.Init(self._arm_callback, 1)
        self.lowstate_subscriber = ChannelSubscriber(self.lowstate_topic, LowState_)
        self.lowstate_subscriber.Init(self._lowstate_callback, 1)
        self.final_publisher = ChannelPublisher(self.final_cmd_topic, LowCmd_)
        self.final_publisher.Init()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)

    def _base_callback(self, msg):
        self.base_cmd = deepcopy(msg)
        self.base_time = time.monotonic()
        self._warned_no_base = False
        self._warned_base_timeout = False
        if not self._reported_base:
            self.get_logger().info(f"Received first base command on {self.base_cmd_topic!r}")
            self._reported_base = True

    def _arm_callback(self, msg):
        self.arm_cmd = deepcopy(msg)
        self.arm_time = time.monotonic()
        self._warned_arm_timeout = False

    def _lowstate_callback(self, msg):
        self.lowstate = msg
        self.lowstate_time = time.monotonic()
        self._warned_no_lowstate = False
        if not self._reported_lowstate:
            self.get_logger().info(f"Received first LowState on {self.lowstate_topic!r}")
            self._reported_lowstate = True

    def _fresh(self, stamp, timeout_s):
        return stamp is not None and (time.monotonic() - stamp) <= timeout_s

    def _latest_lowstate_is_fresh(self):
        return self.lowstate is not None

    def _hold_command_from_lowstate(self):
        cmd = unitree_hg_msg_dds__LowCmd_()
        copy_command_metadata(cmd, lowstate=self.lowstate)
        cmd.mode_pr = 0
        for motor_id in range(G1_NUM_MOTORS):
            set_motor_command(
                cmd,
                motor_id,
                mode=1,
                q=motor_q(self.lowstate, motor_id),
                kp=self.hold_kp,
                kd=self.hold_kd,
            )
        return cmd

    def _apply_base_legs(self, final_cmd):
        if self._fresh(self.base_time, self.base_timeout_s):
            for motor_id in self.leg_motor_ids:
                copy_motor_command(final_cmd, self.base_cmd, motor_id)
            return

        if self.base_cmd is None:
            if not self._warned_no_base:
                self.get_logger().warn(f"Waiting for base command on {self.base_cmd_topic!r}")
                self._warned_no_base = True
            return

        if not self._warned_base_timeout:
            self.get_logger().warn("Base command timed out; holding current leg state.")
            self._warned_base_timeout = True

    def _apply_arm_overlay(self, final_cmd):
        if self.arm_cmd is not None:
            if not self._fresh(self.arm_time, self.arm_timeout_s) and not self._warned_arm_timeout:
                self.get_logger().warn("OpenSoT rt/arm_sdk command timed out; holding last accepted arm command.")
                self._warned_arm_timeout = True
            for motor_id in self.overlay_motor_ids:
                copy_motor_command(final_cmd, self.arm_cmd, motor_id)
            return

        for motor_id, q in zip(self.overlay_motor_ids, self.overlay_nominal_positions):
            set_motor_command(
                final_cmd,
                motor_id,
                mode=1,
                q=q,
                kp=self.hold_kp,
                kd=self.hold_kd,
            )

    def _apply_locked_motors(self, final_cmd):
        for motor_id, q in zip(self.locked_motor_ids, self.locked_positions):
            set_motor_command(
                final_cmd,
                motor_id,
                mode=1,
                q=q,
                kp=self.hold_kp,
                kd=self.hold_kd,
            )

    def _tick(self):
        if not self._latest_lowstate_is_fresh():
            if not self._warned_no_lowstate:
                self.get_logger().warn(f"Waiting for LowState on {self.lowstate_topic!r}")
                self._warned_no_lowstate = True
            return

        final_cmd = self._hold_command_from_lowstate()
        self._apply_base_legs(final_cmd)
        self._apply_arm_overlay(final_cmd)
        self._apply_locked_motors(final_cmd)
        copy_command_metadata(final_cmd, lowstate=self.lowstate, src_cmd=final_cmd)
        final_cmd.crc = self.crc.Crc(final_cmd)
        self.final_publisher.Write(final_cmd)
        if not self._reported_final:
            self.get_logger().info(f"Published first final command to {self.final_cmd_topic!r}")
            self._reported_final = True


def main(args=None):
    rclpy.init(args=args)
    node = ArmSdkLowCmdOverlay()
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
