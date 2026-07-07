"""Safety monitor: stale odometry, emergency stop, timeout checks."""
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from motion_test.transforms import yaw_from_quaternion


class SafetyMonitor:
    """Tracks odometry freshness and emergency-stop state."""

    def __init__(self, node, stale_odom_timeout):
        self._node = node
        self._stale_odom_timeout = stale_odom_timeout
        self._last_odom = None
        self._last_odom_time = None
        self._emergency = False

        self._node.create_subscription(
            Bool, '/emergency_stop', self._emergency_callback, 10)
        self._node.create_subscription(
            Odometry, '/leg_odom2', self._odom_callback, 10)

    def _emergency_callback(self, msg):
        self._emergency = msg.data

    def _odom_callback(self, msg):
        self._last_odom = msg
        self._last_odom_time = self._node.get_clock().now()

    def update_odom(self, odom):
        """Alternative manual update if subscription is not used."""
        self._last_odom = odom
        self._last_odom_time = self._node.get_clock().now()

    def is_safe(self):
        if self._emergency:
            return False
        if self._last_odom_time is None:
            return False
        dt = (self._node.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        if dt > self._stale_odom_timeout:
            return False
        return True

    def is_emergency(self):
        return self._emergency

    def get_pose(self):
        """Return (x, y, yaw) from the latest odometry message, or None."""
        if self._last_odom is None:
            return None
        p = self._last_odom.pose.pose.position
        q = self._last_odom.pose.pose.orientation
        return (p.x, p.y, yaw_from_quaternion(q))

    def reset_emergency(self):
        self._emergency = False
