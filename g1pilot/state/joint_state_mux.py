#!/usr/bin/env python3

from collections import OrderedDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMux(Node):
    def __init__(self):
        super().__init__("joint_state_mux")

        self.declare_parameter("body_topic", "/g1pilot/body/joint_states")
        self.declare_parameter("left_hand_topic", "/g1pilot/dx3/left/joint_states")
        self.declare_parameter("right_hand_topic", "/g1pilot/dx3/right/joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("include_dex3", False)
        self.declare_parameter("publish_rate_hz", 100.0)

        self.body_topic = self.get_parameter("body_topic").get_parameter_value().string_value
        self.left_hand_topic = self.get_parameter("left_hand_topic").get_parameter_value().string_value
        self.right_hand_topic = self.get_parameter("right_hand_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.include_dex3 = bool(self.get_parameter("include_dex3").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if self.publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive")

        self.latest = {
            "body": None,
            "left": None,
            "right": None,
        }

        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.create_subscription(JointState, self.body_topic, self._body_callback, 10)
        if self.include_dex3:
            self.create_subscription(JointState, self.left_hand_topic, self._left_callback, 10)
            self.create_subscription(JointState, self.right_hand_topic, self._right_callback, 10)

        self.create_timer(1.0 / self.publish_rate_hz, self.publish_once)
        self.get_logger().info(
            "Joint state mux publishing "
            f"{self.output_topic!r}; include_dex3={self.include_dex3}"
        )

    def _body_callback(self, msg):
        self.latest["body"] = msg

    def _left_callback(self, msg):
        self.latest["left"] = msg

    def _right_callback(self, msg):
        self.latest["right"] = msg

    @staticmethod
    def _merge_into(merged, msg):
        if msg is None:
            return
        for index, name in enumerate(msg.name):
            position = msg.position[index] if index < len(msg.position) else 0.0
            velocity = msg.velocity[index] if index < len(msg.velocity) else 0.0
            effort = msg.effort[index] if index < len(msg.effort) else 0.0
            merged[name] = (float(position), float(velocity), float(effort))

    def publish_once(self):
        if self.latest["body"] is None:
            return

        merged = OrderedDict()
        self._merge_into(merged, self.latest["body"])
        if self.include_dex3:
            self._merge_into(merged, self.latest["left"])
            self._merge_into(merged, self.latest["right"])

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(merged.keys())
        msg.position = [value[0] for value in merged.values()]
        msg.velocity = [value[1] for value in merged.values()]
        msg.effort = [value[2] for value in merged.values()]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMux()
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
