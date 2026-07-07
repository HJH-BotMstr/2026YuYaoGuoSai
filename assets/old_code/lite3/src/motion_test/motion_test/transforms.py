"""Coordinate frame and angle utilities for Lite3 closed-loop motion control."""
import math
from geometry_msgs.msg import Quaternion


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    """Extract yaw from a quaternion assuming roll=pitch=0."""
    # q.z = sin(yaw/2), q.w = cos(yaw/2)
    return math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)


def quaternion_from_yaw(yaw):
    """Build a quaternion from yaw (roll=pitch=0)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def transform_world_to_body(vx_world, vy_world, yaw):
    """Rotate a world-frame 2D vector into the body frame."""
    vx_body = vx_world * math.cos(yaw) + vy_world * math.sin(yaw)
    vy_body = -vx_world * math.sin(yaw) + vy_world * math.cos(yaw)
    return vx_body, vy_body


def transform_body_to_world(vx_body, vy_body, yaw):
    """Rotate a body-frame 2D vector into the world frame."""
    vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
    vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
    return vx_world, vy_world
