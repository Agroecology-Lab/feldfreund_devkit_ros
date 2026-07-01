#!/usr/bin/env python3
"""
course_over_ground.py
─────────────────────
Converts ublox_dgnss NAV-PVT → sensor_msgs/Imu for FusionCore.

Subscribes : /rover/ubx_nav_pvt   (ublox_ubx_msgs/UBXNavPVT)
Publishes  : /gnss/heading        (sensor_msgs/Imu)

Single-receiver fallback for relposned_heading_shim: when there is no second
F9P and no dual-antenna baseline, this shim derives heading from the receiver's
course over ground (head_mot from NAV-PVT), gated by ground speed and the
receiver's reported heading accuracy.

Course over ground is only meaningful while the robot is moving — at low speeds
the heading vector noise blows up. To keep /gnss/heading populated during
dwells (e.g. topo node arrival pauses) the last accepted heading is re-published
with inflated covariance until either a fresh fix passes the gates or
STALE_TIMEOUT_S elapses.

NAV-PVT head_mot is True North referenced, NED, degrees × 1e-5.
ROS convention: ENU yaw.  NED → ENU: yaw_ENU = π/2 − heading_NED_rad

Running alongside relposned_heading_shim is safe in single-F9P mode (relposned
publishes 0 messages with no baseline). Disable this shim when upgrading to
dual-F9P + RTK so fusioncore receives only one heading source.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ublox_ubx_msgs.msg import UBXNavPVT  # pylint: disable=import-error

# Below this ground speed (mm/s), CoG heading is too noisy to trust as fresh.
# 0.5 m/s ≈ slow walking pace; tune per platform if your robot creeps slower.
_MIN_SPEED_MMPS = 500

# Above this heading accuracy estimate (degrees), reject as too uncertain.
# Open-sky F9P typically reports < 5° while moving; 30° is a generous cap.
_MAX_HEAD_ACC_DEG = 30.0

# How long to keep republishing the last good heading after gates fail (seconds).
# Beyond this, let /gnss/heading go silent so fusioncore stops trusting the cache.
_STALE_TIMEOUT_S = 10.0

# Variance multiplier applied when republishing a stale heading.
# Inflates σ² so fusioncore weights the cached value less while still using it.
_STALE_VAR_INFLATE = 100.0

# Rate at which the stale republisher fires (Hz). 5 Hz matches typical UKF rates.
_STALE_REPUB_HZ = 5.0


class CourseOverGround(Node):
    def __init__(self):
        super().__init__('course_over_ground')

        # When True, a moving-baseline (dual-F9P) heading source is active and
        # publishing to /gnss/heading. This CoG shim must then stay silent so
        # the two nodes don't race on the same topic. The launch file passes
        # this; it was previously declared nowhere and silently ignored, so
        # both shims published simultaneously in dual-antenna mode.
        self.declare_parameter('dual_antenna', False)
        self._dual_antenna = self.get_parameter('dual_antenna').value

        self._pub = self.create_publisher(Imu, '/gnss/heading', 10)
        self._sub = self.create_subscription(
            UBXNavPVT, '/rover/ubx_nav_pvt',
            self._cb, 10)

        self._last_good: Optional[Imu] = None
        self._last_good_t: float = 0.0
        self._pub_count    = 0
        self._stale_count  = 0
        self._reject_count = 0
        self.create_timer(10.0, self._log_stats)
        self.create_timer(1.0 / _STALE_REPUB_HZ, self._republish_stale)
        self.get_logger().info('course_over_ground ready')

    def _cb(self, msg: UBXNavPVT) -> None:
        # Moving-baseline heading is authoritative when present — stay silent.
        if self._dual_antenna:
            return

        # GNSS solution must be valid before we trust any of its fields
        if not msg.gnss_fix_ok or msg.invalid_llh:
            self._reject_count += 1
            return

        # Speed gate — below this CoG noise dominates the signal
        if msg.g_speed < _MIN_SPEED_MMPS:
            self._reject_count += 1
            return

        # Accuracy gate — receiver's own σ on heading
        head_acc_deg = msg.head_acc * 1e-5
        if head_acc_deg > _MAX_HEAD_ACC_DEG:
            self._reject_count += 1
            return

        # head_mot: deg × 1e-5, True North, NED
        heading_rad = math.radians(msg.head_mot * 1e-5)
        # NED → ENU yaw
        yaw_enu = math.pi / 2.0 - heading_rad

        qz = math.sin(yaw_enu / 2.0)
        qw = math.cos(yaw_enu / 2.0)

        out = Imu()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = qz
        out.orientation.w = qw

        acc_rad  = math.radians(head_acc_deg)
        variance = acc_rad ** 2

        # orientation_covariance is row-major 3×3; index [8] = yaw variance.
        # Do NOT use -1 in index [0]: FusionCore's gnss_heading_callback rejects
        # the message when orientation_covariance[0] < 0, silently dropping every
        # heading. Mark roll/pitch unknown with a large positive variance so the
        # validity gate passes; the callback only reads index [8].
        _UNKNOWN = 1.0e6
        out.orientation_covariance = [
            _UNKNOWN, 0.0,      0.0,
            0.0,      _UNKNOWN, 0.0,
            0.0,      0.0,      variance,
        ]

        # Angular velocity and linear acceleration unused — heading only
        out.angular_velocity_covariance[0]    = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        self._pub.publish(out)
        self._pub_count += 1
        self._last_good   = out
        self._last_good_t = self.get_clock().now().nanoseconds * 1e-9

    def _republish_stale(self) -> None:
        """Re-emit the last good heading when fresh data isn't passing the gates.

        Without this, stopping at a node would silence /gnss/heading entirely
        and starve fusioncore's UKF of orientation observations. With it,
        fusioncore keeps a weakly-trusted heading anchor across stops up to
        STALE_TIMEOUT_S, after which the topic goes silent and the filter
        degrades to wheel-odom dead reckoning.
        """
        # In dual-antenna mode this shim is disabled; never republish.
        if self._dual_antenna:
            return
        if self._last_good is None:
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        # Don't double-publish if a fresh message just went out
        if now_s - self._last_good_t < (1.0 / _STALE_REPUB_HZ):
            return
        if now_s - self._last_good_t > _STALE_TIMEOUT_S:
            return

        stale = Imu()
        stale.header.stamp    = self.get_clock().now().to_msg()
        stale.header.frame_id = self._last_good.header.frame_id
        stale.orientation     = self._last_good.orientation
        stale.orientation_covariance = list(self._last_good.orientation_covariance)
        stale.orientation_covariance[8] *= _STALE_VAR_INFLATE
        stale.angular_velocity_covariance[0]    = -1.0
        stale.linear_acceleration_covariance[0] = -1.0
        self._pub.publish(stale)
        self._stale_count += 1

    def _log_stats(self) -> None:
        self.get_logger().info(
            f'course_over_ground: published={self._pub_count}  '
            f'stale_repub={self._stale_count}  '
            f'rejected={self._reject_count} (slow / inaccurate / no fix)')
        self._pub_count    = 0
        self._stale_count  = 0
        self._reject_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = CourseOverGround()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
