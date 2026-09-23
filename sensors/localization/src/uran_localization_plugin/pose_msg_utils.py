import math
from typing import Any, Dict


def quaternion_to_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    return math.degrees(quaternion_to_yaw_rad(x, y, z, w))


def pose_stamped_to_dict(msg) -> Dict[str, Any]:
    orientation = msg.pose.orientation
    position = msg.pose.position
    return {
        'frame_id': msg.header.frame_id,
        'stamp_sec': int(msg.header.stamp.sec),
        'stamp_nanosec': int(msg.header.stamp.nanosec),
        'x': float(position.x),
        'y': float(position.y),
        'z': float(position.z),
        'qx': float(orientation.x),
        'qy': float(orientation.y),
        'qz': float(orientation.z),
        'qw': float(orientation.w),
        'yaw_rad': quaternion_to_yaw_rad(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
        'yaw_deg': quaternion_to_yaw_deg(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
    }
