#!/usr/bin/env python3
import argparse
import sys
import threading

import rclpy
from rclpy.node import Node

from orchard_picker.srv import RawJson


class RawJsonCli(Node):
    def __init__(self, service, wait_service_timeout):
        super().__init__("raw_json_cli")
        self.service_name = service
        self.wait_service_timeout = float(wait_service_timeout)
        self.client = self.create_client(RawJson, service)

    def call(self, request_json, wait_trajectory, device):
        elapsed = 0.0
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            elapsed += 1.0
            self.get_logger().info("Waiting for service {}".format(self.service_name))
            if self.wait_service_timeout > 0.0 and elapsed >= self.wait_service_timeout:
                raise RuntimeError(
                    "service {} is not available. Start rm_json_bridge first, for example: "
                    "ros2 launch orchard_picker driver_only.launch.py".format(self.service_name)
                )

        req = RawJson.Request()
        req.request_json = request_json
        req.wait_for_response = True
        req.wait_for_trajectory = bool(wait_trajectory)
        req.wait_device = int(device)

        done = threading.Event()
        future = self.client.call_async(req)
        future.add_done_callback(lambda _future: done.set())
        while rclpy.ok() and not done.wait(0.1):
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.exception() is not None:
            raise future.exception()
        return future.result()


def main(args=None):
    parser = argparse.ArgumentParser(description="Send one raw JSON command through rm_json_bridge.")
    parser.add_argument("json", help="JSON command, for example '{\"command\":\"get_current_arm_state\"}'")
    parser.add_argument("--service", default="/rm_json_bridge/raw_json")
    parser.add_argument("--wait-trajectory", action="store_true")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument(
        "--wait-service-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for rm_json_bridge service. Use 0 to wait forever.",
    )
    parsed, unknown = parser.parse_known_args()

    rclpy.init(args=args if args is not None else unknown)
    node = RawJsonCli(parsed.service, parsed.wait_service_timeout)
    exit_code = 0
    try:
        resp = node.call(parsed.json, parsed.wait_trajectory, parsed.device)
        print("success:", resp.success)
        print("message:", resp.message)
        print("response_json:", resp.response_json)
    except KeyboardInterrupt:
        print("Interrupted while waiting for {}".format(parsed.service))
        exit_code = 130
    except Exception as exc:
        print("error:", exc)
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
