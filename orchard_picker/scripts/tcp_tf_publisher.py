#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

try:
    from tf2_ros import TransformBroadcaster
except ImportError:  # ROS2 Foxy compatibility
    from tf2_ros.transform_broadcaster import TransformBroadcaster

from orchard_picker.srv import GetArmState


class TcpTfPublisher(Node):
    def __init__(self):
        super().__init__("tcp_tf_publisher")

        self.driver_ns = self._param("driver_ns", "/rm_json_bridge")
        self.base_frame = self._param("base_frame", "base_link")
        self.tcp_frame = self._param("tcp_frame", "tool0")
        self.publish_rate = float(self._param("publish_rate", 5.0))
        self.wait_for_driver = bool(self._param("wait_for_driver", True))

        self.broadcaster = TransformBroadcaster(self)
        self.state_client = self.create_client(GetArmState, self.driver_ns + "/get_arm_state")
        self.pending_future = None

        if self.wait_for_driver:
            while rclpy.ok() and not self.state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service {}".format(self.driver_ns + "/get_arm_state"))

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.2
        self.timer = self.create_timer(period, self._request_state)

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _request_state(self):
        if self.pending_future is not None and not self.pending_future.done():
            return
        if not self.state_client.service_is_ready():
            return

        self.pending_future = self.state_client.call_async(GetArmState.Request())
        self.pending_future.add_done_callback(self._handle_state)

    def _handle_state(self, future):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warning("Failed to get arm state for TCP TF: {}".format(exc))
            return

        if resp is None or not resp.success:
            message = "" if resp is None else resp.message
            self.get_logger().warning("Arm state unavailable for TCP TF: {}".format(message))
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.tcp_frame
        transform.transform.translation.x = resp.pose.position.x
        transform.transform.translation.y = resp.pose.position.y
        transform.transform.translation.z = resp.pose.position.z
        transform.transform.rotation.x = resp.pose.orientation.x
        transform.transform.rotation.y = resp.pose.orientation.y
        transform.transform.rotation.z = resp.pose.orientation.z
        transform.transform.rotation.w = resp.pose.orientation.w

        if abs(transform.transform.rotation.w) < 1e-9:
            transform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = TcpTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
