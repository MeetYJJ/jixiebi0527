#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String

try:
    from rclpy.qos import DurabilityPolicy
except ImportError:  # ROS2 Foxy compatibility
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy

from orchard_picker.protocol import (
    clamp_int,
    joints_deg_to_protocol,
    pose_to_protocol,
    protocol_to_joints_deg,
    protocol_to_pose,
    response_bundle,
)
from orchard_picker.realman_json_client import (
    JsonFrameClient,
    command_ack,
    current_arm_state_ack,
    gripper_ack,
    trajectory_done,
)
from orchard_picker.srv import (
    GetArmState,
    GripperPick,
    GripperPosition,
    GripperRelease,
    MoveJ,
    MovePose,
    RawJson,
    SetToolVoltage,
)


def _transient_local_durability():
    if hasattr(DurabilityPolicy, "TRANSIENT_LOCAL"):
        return DurabilityPolicy.TRANSIENT_LOCAL
    return DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL


class RealManJsonBridge(Node):
    def __init__(self):
        super().__init__("rm_json_bridge")

        self.host = self._param("host", "192.168.1.18")
        self.port = int(self._param("port", 8080))
        self.response_timeout = float(self._param("response_timeout", 5.0))
        self.motion_timeout = float(self._param("motion_timeout", 60.0))
        self.gripper_timeout = float(self._param("gripper_timeout", 20.0))
        self.set_tool_voltage_on_start = bool(self._param("set_tool_voltage_on_start", False))
        self.tool_voltage_type = int(self._param("tool_voltage_type", 3))
        self.gripper_driver = str(self._param("gripper_driver", "inspire_modbus"))
        self.gripper_modbus_device = int(self._param("gripper_modbus_device", 1))
        self.gripper_modbus_type = int(self._param("gripper_modbus_type", 1))
        self.gripper_modbus_baudrate = int(self._param("gripper_modbus_baudrate", 115200))
        self.gripper_modbus_control_address = int(self._param("gripper_modbus_control_address", 10))
        self.gripper_modbus_status_address = int(self._param("gripper_modbus_status_address", 65))
        self.gripper_release_position = int(self._param("gripper_release_position", 1000))
        self.gripper_pick_position = int(self._param("gripper_pick_position", 0))
        self.gripper_position_force = int(self._param("gripper_position_force", 200))
        self.gripper_wait_status = bool(self._param("gripper_wait_status", True))

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = _transient_local_durability()
        self.raw_rx_pub = self.create_publisher(String, "~/raw_rx", 50)
        self.raw_tx_pub = self.create_publisher(String, "~/raw_tx", 50)
        self.connected_pub = self.create_publisher(Bool, "~/connected", latched_qos)

        self.client = JsonFrameClient(
            host=self.host,
            port=self.port,
            connect_timeout=float(self._param("connect_timeout", 3.0)),
            recv_timeout=float(self._param("recv_timeout", 0.2)),
            rx_callback=lambda text: self.raw_rx_pub.publish(String(data=text)),
            tx_callback=lambda text: self.raw_tx_pub.publish(String(data=text)),
            error_callback=lambda message: self.get_logger().warning(
                "rm_json_bridge: {}".format(message)
            ),
        )

        self.create_service(RawJson, "~/raw_json", self.handle_raw_json)
        self.create_service(GetArmState, "~/get_arm_state", self.handle_get_arm_state)
        self.create_service(MoveJ, "~/movej", self.handle_movej)
        self.create_service(MovePose, "~/movej_p", self.handle_movej_p)
        self.create_service(MovePose, "~/movel", self.handle_movel)
        self.create_service(GripperRelease, "~/gripper_release", self.handle_gripper_release)
        self.create_service(GripperPick, "~/gripper_pick", self.handle_gripper_pick)
        self.create_service(GripperPosition, "~/gripper_position", self.handle_gripper_position)
        self.create_service(SetToolVoltage, "~/set_tool_voltage", self.handle_set_tool_voltage)

        if bool(self._param("auto_connect", True)):
            try:
                self.client.connect()
                self.get_logger().info(
                    "Connected to RealMan controller at {}:{}".format(self.host, self.port)
                )
            except Exception as exc:
                self.get_logger().warning("Initial RealMan connection failed: {}".format(exc))

        if self.set_tool_voltage_on_start:
            voltage_type = self.tool_voltage_type
            req = SetToolVoltage.Request()
            req.voltage_type = voltage_type
            resp = self.handle_set_tool_voltage(req, SetToolVoltage.Response())
            if resp.success:
                self.get_logger().info("Tool voltage configured to type {}".format(voltage_type))
            else:
                self.get_logger().warning("Tool voltage setup failed: {}".format(resp.message))

        if self.gripper_driver == "inspire_modbus":
            ok, message = self._configure_tool_rs485()
            if ok:
                self.get_logger().info("Tool RS485 configured for Inspire Modbus gripper")
            else:
                self.get_logger().warning("Tool RS485 setup failed: {}".format(message))

        self.create_timer(1.0, self._publish_connected)

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _publish_connected(self):
        self.connected_pub.publish(Bool(data=bool(self.client.connected)))

    def handle_raw_json(self, req, resp):
        try:
            payload = json.loads(req.request_json)
        except ValueError as exc:
            resp.success = False
            resp.message = "invalid JSON: {}".format(exc)
            resp.response_json = ""
            return resp

        ack_predicate = (lambda _msg: True) if req.wait_for_response else None
        wait_predicate = trajectory_done(req.wait_device) if req.wait_for_trajectory else None
        success, message, ack, done = self._send(
            payload,
            ack_predicate=ack_predicate,
            wait_predicate=wait_predicate,
            wait_timeout=self.motion_timeout,
        )
        resp.success = success
        resp.message = message
        resp.response_json = response_bundle(ack, done)
        return resp

    def handle_get_arm_state(self, _req, resp):
        payload = {"command": "get_current_arm_state"}
        success, message, ack, _done = self._send(
            payload,
            ack_predicate=current_arm_state_ack,
            wait_predicate=None,
        )
        if not success:
            resp.success = False
            resp.message = message
            resp.joint_deg = []
            resp.pose = protocol_to_pose([])
            resp.err = 0
            resp.response_json = response_bundle(ack)
            return resp

        arm_state = ack.get("arm_state", ack)
        joints = protocol_to_joints_deg(arm_state.get("joint", []))
        pose = protocol_to_pose(arm_state.get("pose", []))
        err_value = arm_state.get("err", arm_state.get("arm_err", 0))
        if isinstance(err_value, (list, tuple)):
            err_value = err_value[0] if err_value else 0

        resp.success = True
        resp.message = "ok"
        resp.joint_deg = joints
        resp.pose = pose
        resp.err = int(err_value)
        resp.response_json = response_bundle(ack)
        return resp

    def handle_movej(self, req, resp):
        if len(req.joint_deg) != 6:
            resp.success = False
            resp.message = "joint_deg must contain 6 values"
            resp.response_json = ""
            return resp
        payload = {
            "command": "movej",
            "joint": joints_deg_to_protocol(req.joint_deg),
            "v": clamp_int(req.speed, 1, 100),
            "r": clamp_int(req.blend_radius, 0, 100),
            "trajectory_connect": 0,
        }
        success, message, ack, done = self._send_motion(payload, req.wait, device=0)
        resp.success = success
        resp.message = message
        resp.response_json = response_bundle(ack, done)
        return resp

    def handle_movej_p(self, req, resp):
        return self._handle_pose_motion(req, resp, "movej_p")

    def handle_movel(self, req, resp):
        return self._handle_pose_motion(req, resp, "movel")

    def _handle_pose_motion(self, req, resp, command):
        payload = {
            "command": command,
            "pose": pose_to_protocol(req.pose),
            "v": clamp_int(req.speed, 1, 100),
            "r": clamp_int(req.blend_radius, 0, 100),
            "trajectory_connect": 0,
        }
        success, message, ack, done = self._send_motion(payload, req.wait, device=0)
        resp.success = success
        resp.message = message
        resp.response_json = response_bundle(ack, done)
        return resp

    def handle_gripper_release(self, req, resp):
        if self.gripper_driver == "inspire_modbus":
            success, message, ack, done = self._send_inspire_gripper_target(
                position=self.gripper_release_position,
                speed=req.speed,
                force=self.gripper_position_force,
                wait=req.wait,
                target_name="release",
            )
            resp.success = success
            resp.message = message
            resp.response_json = response_bundle(ack, done)
            return resp

        payload = {
            "command": "set_gripper_release",
            "speed": clamp_int(req.speed, 1, 1000),
            "block": bool(req.wait),
        }
        success, message, ack, done = self._send_gripper(payload, req.wait)
        resp.success = success
        resp.message = message
        resp.response_json = response_bundle(ack, done)
        return resp

    def handle_gripper_pick(self, req, resp):
        if self.gripper_driver == "inspire_modbus":
            success, message, ack, done = self._send_inspire_gripper_target(
                position=self.gripper_pick_position,
                speed=req.speed,
                force=req.force,
                wait=req.wait,
                target_name="pick",
            )
            resp.success = success
            resp.message = message
            resp.response_json = response_bundle(ack, done)
            return resp

        payload = {
            "command": "set_gripper_pick",
            "speed": clamp_int(req.speed, 1, 1000),
            "force": clamp_int(req.force, 50, 1000),
            "block": bool(req.wait),
        }
        success, message, ack, done = self._send_gripper(payload, req.wait)
        resp.success = success
        resp.message = message
        resp.response_json = response_bundle(ack, done)
        return resp

    def handle_gripper_position(self, req, resp):
        if self.gripper_driver == "inspire_modbus":
            success, message, ack, done = self._send_inspire_gripper_target(
                position=req.position,
                speed=500,
                force=self.gripper_position_force,
                wait=req.wait,
                target_name="position",
            )
            resp.success = success
            resp.message = message
            resp.response_json = response_bundle(ack, done)
            return resp

        payload = {
            "command": "set_gripper_position",
            "position": clamp_int(req.position, 1, 1000),
            "block": bool(req.wait),
        }
        success, message, ack, done = self._send_gripper(payload, req.wait)
        resp.success = success
        resp.message = message
        resp.response_json = response_bundle(ack, done)
        return resp

    def handle_set_tool_voltage(self, req, resp):
        payload = {
            "command": "set_tool_voltage",
            "voltage_type": clamp_int(req.voltage_type, 0, 3),
        }
        success, message, ack, _done = self._send(
            payload,
            ack_predicate=command_ack("set_tool_voltage"),
            wait_predicate=None,
        )
        resp.success = success
        resp.message = message
        resp.response_json = response_bundle(ack)
        return resp

    def _send_motion(self, payload, wait, device):
        return self._send(
            payload,
            ack_predicate=command_ack(payload["command"]),
            wait_predicate=trajectory_done(device) if wait else None,
            wait_timeout=self.motion_timeout,
        )

    def _send_gripper(self, payload, wait):
        return self._send(
            payload,
            ack_predicate=gripper_ack,
            wait_predicate=trajectory_done(1) if wait else None,
            wait_timeout=self.gripper_timeout,
        )

    def _configure_tool_rs485(self):
        payload = {
            "command": "set_tool_rs485_mode",
            "mode": 0,
            "baudrate": self.gripper_modbus_baudrate,
        }
        success, message, _ack, _done = self._send(
            payload,
            ack_predicate=command_ack("set_tool_rs485_mode"),
            wait_predicate=None,
        )
        return success, message

    def _send_inspire_gripper_target(self, position, speed, force, wait, target_name):
        position = clamp_int(position, 0, 1000)
        speed = clamp_int(speed, 10, 1000)
        force = clamp_int(force, 100, 1000)
        payload = {
            "command": "write_modbus_rtu_registers",
            "address": self.gripper_modbus_control_address,
            "data": [position, speed, force],
            "device": self.gripper_modbus_device,
            "type": self.gripper_modbus_type,
        }
        success, message, ack, _done = self._send(
            payload,
            ack_predicate=command_ack("write_modbus_rtu_registers"),
            wait_predicate=None,
        )
        if not success or not wait or not self.gripper_wait_status:
            return success, message, ack, None

        done = self._wait_inspire_gripper_done(target_name)
        if done is None:
            return False, "timeout waiting for gripper status", ack, None
        status = int(done.get("data", [0])[0])
        if target_name == "release" and status not in (1, 3):
            return False, "unexpected release status {}".format(status), ack, done
        if target_name == "pick" and status not in (2, 3, 6):
            return False, "unexpected pick status {}".format(status), ack, done
        if target_name == "position" and status in (4, 5):
            return False, "unexpected moving status {}".format(status), ack, done
        return True, "ok", ack, done

    def _wait_inspire_gripper_done(self, target_name):
        deadline = time.time() + self.gripper_timeout
        while time.time() < deadline:
            done = self._read_inspire_gripper_status()
            if done is None:
                time.sleep(0.2)
                continue
            values = done.get("data", [])
            if not values:
                time.sleep(0.2)
                continue
            status = int(values[0])
            if status not in (4, 5):
                return done
            time.sleep(0.2)
        return None

    def _read_inspire_gripper_status(self):
        payload = {
            "command": "read_modbus_rtu_holding_registers",
            "address": self.gripper_modbus_status_address,
            "num": 1,
            "device": self.gripper_modbus_device,
            "type": self.gripper_modbus_type,
        }
        success, _message, ack, _done = self._send(
            payload,
            ack_predicate=command_ack("read_modbus_rtu_holding_registers"),
            wait_predicate=None,
        )
        return ack if success else None

    def _send(self, payload, ack_predicate, wait_predicate, wait_timeout=None):
        try:
            return self.client.send_command(
                payload,
                ack_predicate=ack_predicate,
                ack_timeout=self.response_timeout,
                wait_predicate=wait_predicate,
                wait_timeout=wait_timeout if wait_timeout is not None else self.motion_timeout,
            )
        except Exception as exc:
            self.get_logger().error("RealMan JSON command failed: {}".format(exc))
            return False, str(exc), None, None

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealManJsonBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
