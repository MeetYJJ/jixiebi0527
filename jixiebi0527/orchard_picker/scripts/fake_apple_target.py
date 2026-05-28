#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from orchard_picker.protocol import quaternion_from_euler


class FakeAppleTarget(Node):
    def __init__(self):
        super().__init__("fake_apple_target")
        self.topic = self._param("topic", "/apple/target_pose")
        self.frame_id = self._param("frame_id", "base_link")
        self.rate_hz = float(self._param("rate", 2.0))
        self.x = float(self._param("x", 0.200269))
        self.y = float(self._param("y", -0.219514))
        self.z = float(self._param("z", 0.597973))
        self.roll = float(self._param("roll", 0.782))
        self.pitch = float(self._param("pitch", 0.409))
        self.yaw = float(self._param("yaw", 1.043))

        self.pub = self.create_publisher(PoseStamped, self.topic, 1)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish)

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _publish(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeAppleTarget()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
