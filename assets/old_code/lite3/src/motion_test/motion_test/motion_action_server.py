"""Action server exposing closed-loop motion primitives for Lite3."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

from motion_control_interfaces.action import MoveDistance, RotateAngle, MoveRelative
from motion_test.transforms import normalize_angle
from motion_test.pid_controller import PIDController
from motion_test.safety_monitor import SafetyMonitor


class MotionActionServer(Node):
    def __init__(self):
        super().__init__('motion_action_server')

        # Parameters
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('max_vel_x', 0.3)
        self.declare_parameter('max_vel_y', 0.2)
        self.declare_parameter('max_vel_yaw', 0.5)
        self.declare_parameter('min_vel_x', 0.05)
        self.declare_parameter('min_vel_yaw', 0.05)
        self.declare_parameter('kp_dist', 1.0)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('kp_lateral', 1.0)
        self.declare_parameter('dist_threshold', 0.05)
        self.declare_parameter('yaw_threshold', 0.08)
        self.declare_parameter('lateral_threshold', 0.03)
        self.declare_parameter('stale_odom_timeout', 0.3)
        self.declare_parameter('action_timeout_factor', 2.0)
        self.declare_parameter('max_action_timeout', 30.0)
        self.declare_parameter('cmd_vel_auto_topic', '/cmd_vel_auto')
        self.declare_parameter('cmd_gait_topic', '/cmd_gait')

        self.params = {p: self.get_parameter(p).value for p in [
            'max_vel_x', 'max_vel_y', 'max_vel_yaw',
            'min_vel_x', 'min_vel_yaw',
            'kp_dist', 'kp_yaw', 'kp_lateral',
            'dist_threshold', 'yaw_threshold', 'lateral_threshold',
        ]}
        self.control_rate = self.get_parameter('control_rate').value
        self.stale_odom_timeout = self.get_parameter('stale_odom_timeout').value
        self.action_timeout_factor = self.get_parameter('action_timeout_factor').value
        self.max_action_timeout = self.get_parameter('max_action_timeout').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_auto_topic').value
        self.cmd_gait_topic = self.get_parameter('cmd_gait_topic').value

        self.controller = PIDController(self.params)
        self.safety = SafetyMonitor(self, self.stale_odom_timeout)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cmd_gait_pub = self.create_publisher(String, self.cmd_gait_topic, 10)

        # Action servers
        self._move_distance_server = ActionServer(
            self, MoveDistance, 'move_distance',
            self.execute_move_distance,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)

        self._rotate_angle_server = ActionServer(
            self, RotateAngle, 'rotate_angle',
            self.execute_rotate_angle,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)

        self._move_relative_server = ActionServer(
            self, MoveRelative, 'move_relative',
            self.execute_move_relative,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)

        self._current_goal_handle = None

        self.get_logger().info('Motion action server initialized')

    # ------------------------------------------------------------------
    # Action boilerplate
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('New goal accepted, canceling previous goal')
            self._current_goal_handle.abort()
        self._current_goal_handle = goal_handle
        goal_handle.execute()

    def _zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def _set_gait(self, gait_name):
        if gait_name:
            self.cmd_gait_pub.publish(String(data=gait_name))

    def _sleep_period(self):
        return 1.0 / self.control_rate

    def _action_timeout(self, expected_duration):
        """Conservative timeout: factor * expected time, capped at max_action_timeout."""
        return min(expected_duration * self.action_timeout_factor, self.max_action_timeout)

    def _wait_for_pose(self, timeout=2.0):
        """Block briefly until we have a fresh odometry reading."""
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < timeout:
            if self.safety.get_pose() is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    # ------------------------------------------------------------------
    # MoveDistance
    # ------------------------------------------------------------------
    def execute_move_distance(self, goal_handle):
        distance = goal_handle.request.distance
        self.get_logger().info(f'MoveDistance goal: {distance:.3f} m')

        if not self._wait_for_pose():
            self._zero_velocity()
            goal_handle.abort()
            return MoveDistance.Result(success=False, message='No odometry available')

        if not goal_handle.is_active:
            self._zero_velocity()
            return MoveDistance.Result(success=False, message='Preempted during initialization')

        start_pose = self.safety.get_pose()
        # Conservative timeout estimate.
        expected_time = abs(distance) / max(self.params['max_vel_x'], 0.01) + 5.0
        deadline = self.get_clock().now() + Duration(seconds=self._action_timeout(expected_time))

        self._set_gait('slow')

        while rclpy.ok():
            if not goal_handle.is_active:
                self._zero_velocity()
                return MoveDistance.Result(success=False, message='Preempted by another goal')

            if goal_handle.is_cancel_requested:
                self._zero_velocity()
                goal_handle.canceled()
                return MoveDistance.Result(success=False, message='Canceled')

            if not self.safety.is_safe():
                self._zero_velocity()
                goal_handle.abort()
                return MoveDistance.Result(success=False, message='Safety violation')

            if self.get_clock().now() > deadline:
                self._zero_velocity()
                goal_handle.abort()
                return MoveDistance.Result(success=False, message='Timeout')

            current_pose = self.safety.get_pose()
            v_x, v_y, omega, e_dist, e_yaw = self.controller.compute_move_distance_vel(
                start_pose, current_pose, distance)

            if abs(e_dist) < self.params['dist_threshold'] and abs(e_yaw) < self.params['yaw_threshold']:
                self._zero_velocity()
                goal_handle.succeed()
                return MoveDistance.Result(success=True, message='Reached target distance')

            self._publish_cmd_vel(v_x, v_y, omega)

            feedback = MoveDistance.Feedback()
            feedback.remaining_distance = float(e_dist)
            feedback.current_x = float(current_pose[0])
            feedback.current_y = float(current_pose[1])
            feedback.current_yaw = float(current_pose[2])
            goal_handle.publish_feedback(feedback)

            rclpy.spin_once(self, timeout_sec=self._sleep_period())

    # ------------------------------------------------------------------
    # RotateAngle
    # ------------------------------------------------------------------
    def execute_rotate_angle(self, goal_handle):
        angle = goal_handle.request.angle
        self.get_logger().info(f'RotateAngle goal: {math.degrees(angle):.1f} deg')

        if not self._wait_for_pose():
            self._zero_velocity()
            goal_handle.abort()
            return RotateAngle.Result(success=False, message='No odometry available')

        if not goal_handle.is_active:
            self._zero_velocity()
            return RotateAngle.Result(success=False, message='Preempted during initialization')

        start_pose = self.safety.get_pose()
        start_yaw = start_pose[2]
        expected_time = abs(angle) / max(self.params['max_vel_yaw'], 0.01) + 5.0
        deadline = self.get_clock().now() + Duration(seconds=self._action_timeout(expected_time))

        self._set_gait('slow')

        while rclpy.ok():
            if not goal_handle.is_active:
                self._zero_velocity()
                return RotateAngle.Result(success=False, message='Preempted by another goal')

            if goal_handle.is_cancel_requested:
                self._zero_velocity()
                goal_handle.canceled()
                return RotateAngle.Result(success=False, message='Canceled')

            if not self.safety.is_safe():
                self._zero_velocity()
                goal_handle.abort()
                return RotateAngle.Result(success=False, message='Safety violation')

            if self.get_clock().now() > deadline:
                self._zero_velocity()
                goal_handle.abort()
                return RotateAngle.Result(success=False, message='Timeout')

            current_yaw = self.safety.get_pose()[2]
            v_x, v_y, omega, e_yaw = self.controller.compute_rotate_vel(
                start_yaw, current_yaw, angle)

            if abs(e_yaw) < self.params['yaw_threshold']:
                self._zero_velocity()
                goal_handle.succeed()
                return RotateAngle.Result(success=True, message='Reached target angle')

            self._publish_cmd_vel(v_x, v_y, omega)

            feedback = RotateAngle.Feedback()
            feedback.remaining_angle = float(e_yaw)
            feedback.current_yaw = float(current_yaw)
            goal_handle.publish_feedback(feedback)

            rclpy.spin_once(self, timeout_sec=self._sleep_period())

    # ------------------------------------------------------------------
    # MoveRelative
    # ------------------------------------------------------------------
    def execute_move_relative(self, goal_handle):
        x = goal_handle.request.x
        y = goal_handle.request.y
        yaw = goal_handle.request.yaw
        self.get_logger().info(f'MoveRelative goal: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')

        if not self._wait_for_pose():
            self._zero_velocity()
            goal_handle.abort()
            return MoveRelative.Result(success=False, message='No odometry available')

        if not goal_handle.is_active:
            self._zero_velocity()
            return MoveRelative.Result(success=False, message='Preempted during initialization')

        start_pose = self.safety.get_pose()
        target_rel = (x, y, yaw)

        expected_dist = math.hypot(x, y)
        expected_time = expected_dist / max(self.params['max_vel_x'], 0.01) + abs(yaw) / max(self.params['max_vel_yaw'], 0.01) + 5.0
        deadline = self.get_clock().now() + Duration(seconds=self._action_timeout(expected_time))

        self._set_gait('slow')

        while rclpy.ok():
            if not goal_handle.is_active:
                self._zero_velocity()
                return MoveRelative.Result(success=False, message='Preempted by another goal')

            if goal_handle.is_cancel_requested:
                self._zero_velocity()
                goal_handle.canceled()
                return MoveRelative.Result(success=False, message='Canceled')

            if not self.safety.is_safe():
                self._zero_velocity()
                goal_handle.abort()
                return MoveRelative.Result(success=False, message='Safety violation')

            if self.get_clock().now() > deadline:
                self._zero_velocity()
                goal_handle.abort()
                return MoveRelative.Result(success=False, message='Timeout')

            current_pose = self.safety.get_pose()
            v_x, v_y, omega, e_dist, e_yaw_final = self.controller.compute_relative_pose_vel(
                start_pose, current_pose, target_rel)

            if e_dist < self.params['dist_threshold'] and abs(e_yaw_final) < self.params['yaw_threshold']:
                self._zero_velocity()
                goal_handle.succeed()
                return MoveRelative.Result(success=True, message='Reached target pose')

            self._publish_cmd_vel(v_x, v_y, omega)

            feedback = MoveRelative.Feedback()
            feedback.remaining_distance = float(e_dist)
            feedback.remaining_angle = float(e_yaw_final)
            feedback.current_x = float(current_pose[0])
            feedback.current_y = float(current_pose[1])
            feedback.current_yaw = float(current_pose[2])
            goal_handle.publish_feedback(feedback)

            rclpy.spin_once(self, timeout_sec=self._sleep_period())

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_cmd_vel(self, vx, vy, omega):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
