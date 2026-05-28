#!/usr/bin/env python3
import math
import threading
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers geometry message transforms
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    from rclpy.qos import DurabilityPolicy
except ImportError:  # ROS2 Foxy compatibility
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy

from orchard_picker.protocol import quaternion_from_euler
from orchard_picker.srv import (
    GetArmState,
    GripperPick,
    GripperRelease,
    MoveJ,
    MovePose,
)


def _transient_local_durability():
    if hasattr(DurabilityPolicy, "TRANSIENT_LOCAL"):
        return DurabilityPolicy.TRANSIENT_LOCAL
    return DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL


class PickTaskManager(Node):
    def __init__(self):
        super().__init__("pick_task_manager")
        self.callback_group = ReentrantCallbackGroup()

        self.base_frame = self._param("base_frame", "base_link")
        self.use_tf = bool(self._param("use_tf", True))
        self.target_timeout = float(self._param("target_timeout", 2.0))
        self.transform_timeout = float(self._param("transform_timeout", 0.5))
        self.service_call_timeout = float(self._param("service_call_timeout", 90.0))

        self.approach_vector = self._normalized(
            self._param("approach_vector_base", [1.0, 0.0, 0.0])
        )
        self.approach_distance = float(self._param("approach_distance", 0.05))
        self.retreat_distance = float(self._param("retreat_distance", 0.05))
        self.grasp_offset = [float(v) for v in self._param("grasp_offset_base", [0.0, 0.0, 0.0])]
        self.safe_tcp_pose_m_rpy_rad = [
            float(v)
            for v in self._param(
                "safe_tcp_pose_m_rpy_rad",
                [0.150269, -0.219514, 0.597973, 0.782, 0.409, 1.043],
            )
        ]
        self.tcp_rpy = [float(v) for v in self._param("tcp_rpy_rad", [0.782, 0.409, 1.043])]

        self.workspace_min = [float(v) for v in self._param("workspace_min", [0.05, -0.50, 0.02])]
        self.workspace_max = [float(v) for v in self._param("workspace_max", [0.80, 0.50, 0.80])]
        self.home_joint_deg = [
            float(v)
            for v in self._param(
                "home_joint_deg",
                [107.149, -84.991, 112.484, -130.772, 64.784, 85.805],
            )
        ]

        self.movej_p_speed = int(self._param("movej_p_speed", 10))
        self.movel_speed = int(self._param("movel_speed", 8))
        self.home_speed = int(self._param("home_speed", 10))
        self.twist_speed = int(self._param("twist_speed", 8))
        self.blend_radius = int(self._param("blend_radius", 0))

        self.gripper_release_speed = int(self._param("gripper_release_speed", 300))
        self.gripper_pick_speed = int(self._param("gripper_pick_speed", 200))
        self.gripper_pick_force = int(self._param("gripper_pick_force", 150))

        self.twist_joint_index = int(self._param("twist_joint_index", 5))
        self.twist_delta_deg = float(self._param("twist_delta_deg", 45.0))
        self.twist_min_deg = float(self._param("twist_min_deg", -360.0))
        self.twist_max_deg = float(self._param("twist_max_deg", 360.0))
        self.sleep_after_grip = float(self._param("sleep_after_grip", 0.3))
        self.sleep_after_twist = float(self._param("sleep_after_twist", 0.2))

        self.return_home_on_failure = bool(self._param("return_home_on_failure", True))
        self.release_on_failure = bool(self._param("release_on_failure", False))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_target = None
        self.latest_target_kind = None
        self.lock = threading.RLock()
        self.active = False
        self.abort_requested = False

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = _transient_local_durability()
        self.state_pub = self.create_publisher(String, "~/state", latched_qos)
        self.active_pub = self.create_publisher(Bool, "~/active", latched_qos)

        pose_topic = self._param("target_pose_topic", "/apple/target_pose")
        point_topic = self._param("target_point_topic", "/apple/target_point")
        self.create_subscription(
            PoseStamped,
            pose_topic,
            self._pose_target_cb,
            1,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PointStamped,
            point_topic,
            self._point_target_cb,
            1,
            callback_group=self.callback_group,
        )

        self._connect_services()

        self.create_service(Trigger, "~/run_once", self.handle_run_once, callback_group=self.callback_group)
        self.create_service(Trigger, "~/start", self.handle_start, callback_group=self.callback_group)
        self.create_service(Trigger, "~/abort", self.handle_abort, callback_group=self.callback_group)
        self.create_service(Trigger, "~/home", self.handle_home, callback_group=self.callback_group)

        self.start_on_target = bool(self._param("start_on_target", False))
        self._set_state("IDLE")

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _connect_services(self):
        prefix = self._param("driver_ns", "/rm_json_bridge")
        wait_for_driver = bool(self._param("wait_for_driver", True))
        service_names = {
            "get_state": prefix + "/get_arm_state",
            "movej": prefix + "/movej",
            "movej_p": prefix + "/movej_p",
            "movel": prefix + "/movel",
            "gripper_release": prefix + "/gripper_release",
            "gripper_pick": prefix + "/gripper_pick",
        }

        self.get_state_srv = self.create_client(
            GetArmState, service_names["get_state"], callback_group=self.callback_group
        )
        self.movej_srv = self.create_client(MoveJ, service_names["movej"], callback_group=self.callback_group)
        self.movej_p_srv = self.create_client(
            MovePose, service_names["movej_p"], callback_group=self.callback_group
        )
        self.movel_srv = self.create_client(MovePose, service_names["movel"], callback_group=self.callback_group)
        self.gripper_release_srv = self.create_client(
            GripperRelease, service_names["gripper_release"], callback_group=self.callback_group
        )
        self.gripper_pick_srv = self.create_client(
            GripperPick, service_names["gripper_pick"], callback_group=self.callback_group
        )

        if wait_for_driver:
            for name, client in (
                (service_names["get_state"], self.get_state_srv),
                (service_names["movej"], self.movej_srv),
                (service_names["movej_p"], self.movej_p_srv),
                (service_names["movel"], self.movel_srv),
                (service_names["gripper_release"], self.gripper_release_srv),
                (service_names["gripper_pick"], self.gripper_pick_srv),
            ):
                while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info("Waiting for service {}".format(name))

    def _pose_target_cb(self, msg):
        with self.lock:
            self.latest_target = msg
            self.latest_target_kind = "pose"
        if self.start_on_target:
            self._start_async()

    def _point_target_cb(self, msg):
        with self.lock:
            self.latest_target = msg
            self.latest_target_kind = "point"
        if self.start_on_target:
            self._start_async()

    def handle_run_once(self, _req, resp):
        if not self._claim_active():
            resp.success = False
            resp.message = "picker is already active"
            return resp
        try:
            resp.success, resp.message = self._execute_pick()
            return resp
        finally:
            self._release_active()

    def handle_start(self, _req, resp):
        if not self._start_async():
            resp.success = False
            resp.message = "picker is already active"
            return resp
        resp.success = True
        resp.message = "started"
        return resp

    def handle_abort(self, _req, resp):
        with self.lock:
            self.abort_requested = True
        self._set_state("ABORT_REQUESTED")
        resp.success = True
        resp.message = "abort requested"
        return resp

    def handle_home(self, _req, resp):
        if not self._claim_active():
            resp.success = False
            resp.message = "picker is already active"
            return resp
        try:
            resp.success, resp.message = self._home()
            return resp
        finally:
            self._release_active()

    def _start_async(self):
        if not self._claim_active():
            return False
        thread = threading.Thread(target=self._execute_pick_async, daemon=True)
        thread.start()
        return True

    def _execute_pick_async(self):
        try:
            success, message = self._execute_pick()
            if success:
                self.get_logger().info("Pick task finished: {}".format(message))
            else:
                self.get_logger().error("Pick task failed: {}".format(message))
        finally:
            self._release_active()

    def _claim_active(self):
        with self.lock:
            if self.active:
                return False
            self.active = True
            self.abort_requested = False
            self.active_pub.publish(Bool(data=True))
            return True

    def _release_active(self):
        with self.lock:
            self.active = False
            self.active_pub.publish(Bool(data=False))

    def _execute_pick(self):
        try:
            target = self._get_target_in_base()
            self._check_workspace(target)
            pre_grasp, grasp, retreat = self._build_pick_poses(target)

            self._step("OPEN_GRIPPER", lambda: self._gripper_release())
            self._step("MOVE_PRE_GRASP", lambda: self._move_pose(self.movej_p_srv, pre_grasp, self.movej_p_speed))
            self._step("MOVE_GRASP", lambda: self._move_pose(self.movel_srv, grasp, self.movel_speed))
            self._step("CLOSE_GRIPPER", lambda: self._gripper_pick())
            if self.sleep_after_grip > 0:
                time.sleep(self.sleep_after_grip)
            self._step("TWIST_J6", self._twist_joint)
            if self.sleep_after_twist > 0:
                time.sleep(self.sleep_after_twist)
            self._step("RETREAT", lambda: self._move_pose(self.movel_srv, retreat, self.movel_speed))
            self._step("HOME", self._home)
            self._step("RELEASE_GRIPPER", lambda: self._gripper_release())
            self._set_state("DONE")
            return True, "pick sequence completed"
        except Exception as exc:
            self.get_logger().error("Pick sequence error: {}".format(exc))
            self._set_state("FAILED")
            self._failure_recovery()
            return False, str(exc)

    def _failure_recovery(self):
        if self.return_home_on_failure:
            ok, message = self._home()
            self.get_logger().warning("Failure recovery home: {} {}".format(ok, message))
        if self.release_on_failure:
            ok, message = self._gripper_release()
            self.get_logger().warning("Failure recovery release: {} {}".format(ok, message))

    def _step(self, state, action):
        self._check_abort()
        self._set_state(state)
        ok, message = action()
        if not ok:
            raise RuntimeError("{} failed: {}".format(state, message))

    def _check_abort(self):
        with self.lock:
            if self.abort_requested:
                raise RuntimeError("aborted by user")

    def _get_target_in_base(self):
        with self.lock:
            target = self.latest_target
            kind = self.latest_target_kind

        if target is None:
            raise RuntimeError("no apple target has been received")

        stamp = target.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
            if age > self.target_timeout:
                raise RuntimeError("apple target is stale: {:.2f}s".format(age))

        if not self.use_tf or target.header.frame_id in ("", self.base_frame):
            if kind == "pose":
                return [
                    target.pose.position.x,
                    target.pose.position.y,
                    target.pose.position.z,
                ]
            return [target.point.x, target.point.y, target.point.z]

        try:
            transformed = self.tf_buffer.transform(
                target,
                self.base_frame,
                timeout=Duration(seconds=self.transform_timeout),
            )
        except Exception as exc:
            raise RuntimeError("failed to transform target to {}: {}".format(self.base_frame, exc))

        if kind == "pose":
            return [
                transformed.pose.position.x,
                transformed.pose.position.y,
                transformed.pose.position.z,
            ]
        return [transformed.point.x, transformed.point.y, transformed.point.z]

    def _check_workspace(self, point):
        for i, axis in enumerate(("x", "y", "z")):
            if point[i] < self.workspace_min[i] or point[i] > self.workspace_max[i]:
                raise RuntimeError(
                    "target {}={} outside workspace [{}, {}]".format(
                        axis, point[i], self.workspace_min[i], self.workspace_max[i]
                    )
                )

    def _build_pick_poses(self, target):
        grasp_point = [target[i] + self.grasp_offset[i] for i in range(3)]
        pre_point = [
            grasp_point[i] - self.approach_vector[i] * self.approach_distance
            for i in range(3)
        ]
        retreat_point = [
            grasp_point[i] - self.approach_vector[i] * self.retreat_distance
            for i in range(3)
        ]
        return (
            self._make_pose(pre_point),
            self._make_pose(grasp_point),
            self._make_pose(retreat_point),
        )

    def _make_pose(self, point):
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        q = quaternion_from_euler(self.tcp_rpy[0], self.tcp_rpy[1], self.tcp_rpy[2])
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def _move_pose(self, client, pose, speed):
        req = MovePose.Request()
        req.pose = pose
        req.speed = int(speed)
        req.blend_radius = int(self.blend_radius)
        req.wait = True
        try:
            resp = self._call_service(client, req)
            return resp.success, resp.message
        except Exception as exc:
            return False, str(exc)

    def _home(self):
        req = MoveJ.Request()
        req.joint_deg = list(self.home_joint_deg)
        req.speed = int(self.home_speed)
        req.blend_radius = int(self.blend_radius)
        req.wait = True
        try:
            resp = self._call_service(self.movej_srv, req)
            return resp.success, resp.message
        except Exception as exc:
            return False, str(exc)

    def _gripper_release(self):
        req = GripperRelease.Request()
        req.speed = int(self.gripper_release_speed)
        req.wait = True
        try:
            resp = self._call_service(self.gripper_release_srv, req)
            return resp.success, resp.message
        except Exception as exc:
            return False, str(exc)

    def _gripper_pick(self):
        req = GripperPick.Request()
        req.speed = int(self.gripper_pick_speed)
        req.force = int(self.gripper_pick_force)
        req.wait = True
        try:
            resp = self._call_service(self.gripper_pick_srv, req)
            return resp.success, resp.message
        except Exception as exc:
            return False, str(exc)

    def _twist_joint(self):
        try:
            state = self._call_service(self.get_state_srv, GetArmState.Request())
        except Exception as exc:
            return False, str(exc)
        if not state.success:
            return False, state.message
        joints = list(state.joint_deg)
        if len(joints) <= self.twist_joint_index:
            return False, "arm state did not return joint {}".format(self.twist_joint_index + 1)
        joints[self.twist_joint_index] += self.twist_delta_deg
        if joints[self.twist_joint_index] < self.twist_min_deg or joints[self.twist_joint_index] > self.twist_max_deg:
            return False, "twist target joint limit exceeded: {:.3f} deg".format(joints[self.twist_joint_index])

        req = MoveJ.Request()
        req.joint_deg = joints
        req.speed = int(self.twist_speed)
        req.blend_radius = int(self.blend_radius)
        req.wait = True
        try:
            resp = self._call_service(self.movej_srv, req)
            return resp.success, resp.message
        except Exception as exc:
            return False, str(exc)

    def _call_service(self, client, request):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(self.service_call_timeout):
            future.cancel()
            raise RuntimeError("timeout waiting for service response")
        if future.exception() is not None:
            raise future.exception()
        result = future.result()
        if result is None:
            raise RuntimeError("service returned no response")
        return result

    def _set_state(self, state):
        self.get_logger().info("picker state: {}".format(state))
        self.state_pub.publish(String(data=state))

    @staticmethod
    def _normalized(values):
        vector = [float(v) for v in values]
        norm = math.sqrt(sum(v * v for v in vector))
        if norm < 1e-9:
            raise ValueError("approach_vector_base must not be zero")
        return [v / norm for v in vector]


def main(args=None):
    rclpy.init(args=args)
    node = PickTaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
