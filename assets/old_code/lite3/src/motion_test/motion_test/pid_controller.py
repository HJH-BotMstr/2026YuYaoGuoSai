"""PID-style velocity generators for closed-loop motion primitives."""
import math
from motion_test.transforms import normalize_angle


def _clamp(value, limit):
    """Clamp |value| to |limit|, preserving sign."""
    return max(-abs(limit), min(abs(limit), value))


def _apply_min(value, min_val, max_val):
    """Apply minimum command magnitude when outside the dead zone."""
    if abs(value) < 1e-6:
        return 0.0
    clamped = _clamp(value, max_val)
    if abs(clamped) < min_val:
        return math.copysign(min_val, clamped)
    return clamped


class PIDController:
    """Simple proportional controller with limits; intentionally no integral term
    because leg odometry drifts and integrating it would cause windup."""

    def __init__(self, params):
        self.p = params

    def compute_move_distance_vel(self, start_pose, current_pose, distance):
        """Travel `distance` meters along the starting heading.

        Returns (vx_body, vy_body, omega, e_dist, e_yaw).
        """
        x0, y0, yaw0 = start_pose
        xc, yc, yawc = current_pose

        # Distance traveled along the start heading.
        traveled = (xc - x0) * math.cos(yaw0) + (yc - y0) * math.sin(yaw0)
        e_dist = distance - traveled

        # Heading error relative to the start heading.
        e_yaw = normalize_angle(yaw0 - yawc)

        v_x = self.p['kp_dist'] * e_dist
        v_x = _apply_min(v_x, self.p['min_vel_x'], self.p['max_vel_x'])

        # Small lateral correction to stay on the line.
        lateral_error = -(xc - x0) * math.sin(yaw0) + (yc - y0) * math.cos(yaw0)
        v_y = _clamp(self.p['kp_lateral'] * lateral_error, self.p['max_vel_y'])

        omega = _clamp(self.p['kp_yaw'] * e_yaw, self.p['max_vel_yaw'])

        return v_x, v_y, omega, e_dist, e_yaw

    def compute_rotate_vel(self, start_yaw, current_yaw, angle):
        """Rotate in place by `angle` radians from start_yaw.

        Returns (vx_body, vy_body, omega, e_yaw).
        """
        target_yaw = normalize_angle(start_yaw + angle)
        e_yaw = normalize_angle(target_yaw - current_yaw)

        omega = _apply_min(
            self.p['kp_yaw'] * e_yaw,
            self.p['min_vel_yaw'],
            self.p['max_vel_yaw']
        )

        return 0.0, 0.0, omega, e_yaw

    def compute_relative_pose_vel(self, start_pose, current_pose, target_rel):
        """Move to a pose (x, y, yaw) relative to the start pose.

        target_rel: (dx_forward, dy_left, dyaw_ccw)
        Returns (vx_body, vy_body, omega, e_dist, e_yaw_final).
        """
        x0, y0, yaw0 = start_pose
        xc, yc, yawc = current_pose
        dx_rel, dy_rel, dyaw_rel = target_rel

        # Target position in world frame.
        dx_world = dx_rel * math.cos(yaw0) - dy_rel * math.sin(yaw0)
        dy_world = dx_rel * math.sin(yaw0) + dy_rel * math.cos(yaw0)
        x_t = x0 + dx_world
        y_t = y0 + dy_world
        yaw_t = normalize_angle(yaw0 + dyaw_rel)

        e_x = x_t - xc
        e_y = y_t - yc
        e_dist = math.hypot(e_x, e_y)

        # Direction to the target in world frame.
        phi = math.atan2(e_y, e_x)
        e_yaw_to_target = normalize_angle(phi - yawc)
        e_yaw_final = normalize_angle(yaw_t - yawc)

        # Blend from facing the target to facing the final heading as we approach.
        near_threshold = max(2.0 * self.p['dist_threshold'], 0.1)
        blend = 0.0 if e_dist < near_threshold else 1.0
        e_yaw = blend * e_yaw_to_target + (1.0 - blend) * e_yaw_final

        # Velocity vector in world frame, then transform to body frame.
        v_world = self.p['kp_dist'] * min(e_dist, 1.0)
        v_world_x = v_world * math.cos(phi)
        v_world_y = v_world * math.sin(phi)

        vx_body = v_world_x * math.cos(yawc) + v_world_y * math.sin(yawc)
        vy_body = -v_world_x * math.sin(yawc) + v_world_y * math.cos(yawc)

        vx_body = _apply_min(vx_body, self.p['min_vel_x'], self.p['max_vel_x'])
        vy_body = _clamp(vy_body, self.p['max_vel_y'])
        omega = _apply_min(
            self.p['kp_yaw'] * e_yaw,
            self.p['min_vel_yaw'],
            self.p['max_vel_yaw']
        )

        return vx_body, vy_body, omega, e_dist, e_yaw_final
