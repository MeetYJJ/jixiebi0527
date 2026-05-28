import json
import math

from geometry_msgs.msg import Pose


def clamp_int(value, lower, upper):
    return max(lower, min(upper, int(value)))


def deg_to_protocol(deg):
    """Degrees to RealMan 0.001 degree integer."""
    return int(round(float(deg) * 1000.0))


def protocol_to_deg(value):
    return float(value) / 1000.0


def meter_to_protocol(meter):
    """Meters to RealMan 0.001 mm integer."""
    return int(round(float(meter) * 1000000.0))


def protocol_to_meter(value):
    return float(value) / 1000000.0


def rad_to_protocol(rad):
    """Radians to RealMan 0.001 rad integer."""
    return int(round(float(rad) * 1000.0))


def protocol_to_rad(value):
    return float(value) / 1000.0


def euler_from_quaternion_xyzw(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose_to_protocol(pose):
    q = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    norm = math.sqrt(sum(item * item for item in q))
    if norm < 1e-9:
        roll, pitch, yaw = 0.0, 0.0, 0.0
    else:
        q = [item / norm for item in q]
        roll, pitch, yaw = euler_from_quaternion_xyzw(q)

    return [
        meter_to_protocol(pose.position.x),
        meter_to_protocol(pose.position.y),
        meter_to_protocol(pose.position.z),
        rad_to_protocol(roll),
        rad_to_protocol(pitch),
        rad_to_protocol(yaw),
    ]


def protocol_to_pose(values):
    pose = Pose()
    if len(values) < 6:
        return pose

    pose.position.x = protocol_to_meter(values[0])
    pose.position.y = protocol_to_meter(values[1])
    pose.position.z = protocol_to_meter(values[2])
    q = quaternion_from_euler(
        protocol_to_rad(values[3]),
        protocol_to_rad(values[4]),
        protocol_to_rad(values[5]),
    )
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def joints_deg_to_protocol(joint_deg):
    return [deg_to_protocol(value) for value in joint_deg]


def protocol_to_joints_deg(joint_values):
    return [protocol_to_deg(value) for value in joint_values]


def dumps_compact(payload):
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def response_bundle(ack=None, done=None):
    data = {}
    if ack is not None:
        data["ack"] = ack
    if done is not None:
        data["done"] = done
    return dumps_compact(data)
