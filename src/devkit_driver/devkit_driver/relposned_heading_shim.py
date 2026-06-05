#!/usr/bin/env python3
"""
relposned_heading_shim.py
─────────────────────────
Converts ublox_dgnss NAV-RELPOSNED → sensor_msgs/Imu for FusionCore.
Subscribes : /rover/ubx_nav_rel_pos_ned  (ublox_ubx_msgs/UBXNavRelPosNED)
Publishes  : /gnss/heading               (sensor_msgs/Imu)
FusionCore subscribes to gnss.heading_topic as sensor_msgs/Imu, extracting
yaw from the orientation quaternion. orientation_covariance[8] carries σ²
of the yaw estimate. All other fields (linear_acceleration, angular_velocity,
their covariances, roll, pitch) are left zero — this message carries heading
only.
Only publishes when rel_pos_valid AND rel_pos_heading_valid are both set,
and the antenna baseline passes the minimum length check.
NAV-RELPOSNED heading is True North referenced, degrees × 1e-5.
ROS convention: ENU yaw.  NED → ENU: yaw_ENU = π/2 - heading_NED_rad
"""
import math
import rclpy
from rclpy.node import Node
from ublox_ubx_msgs.msg import UBXNavRelPosNED  # pylint: disable=import-error
from sensor_msgs.msg import Imu

# Minimum antenna separation in metres — below this the heading geometry is unreliable
_MIN_BASELINE_M = 0.3


class RelPosnedHeadingShim(Node):
    def __init__(self):
        super().__init__('relposned_heading_shim')
        self._pub = self.create_publisher(Imu, '/gnss/heading', 10)
        self._sub = self.create_subscription(
            UBXNavRelPosNED, '/rover/ubx_nav_rel_pos_ned',
            self._cb, 10)
        self._pub_count    = 0
        self._reject_count = 0
        self.create_timer(10.0, self._log_stats)
        self.get_logger().info('relposned_heading_shim ready')

    def _cb(self, msg: UBXNavRelPosNED) -> None:
        # Both validity flags must be set before trusting the heading
        if not msg.rel_pos_valid or not msg.rel_pos_heading_valid:
            self._reject_count += 1
            return

        # rel_pos_length is in cm in the ublox_dgnss ROS message
        if (msg.rel_pos_length / 100.0) < _MIN_BASELINE_M:
            self._reject_count += 1
            return

        # rel_pos_heading: degrees × 1e-5, True North, NED convention
        heading_rad = math.radians(msg.rel_pos_heading * 1e-5)
        # Convert NED → ENU: yaw_ENU = π/2 − heading_NED
        yaw_enu = math.pi / 2.0 - heading_rad

        # Pack yaw-only into quaternion (roll=0, pitch=0)
        qz = math.sin(yaw_enu / 2.0)
        qw = math.cos(yaw_enu / 2.0)

        out = Imu()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = qz
        out.orientation.w = qw

        # acc_heading: degrees × 1e-5 accuracy estimate → σ²
        acc_rad  = math.radians(msg.acc_heading * 1e-5)
        variance = acc_rad ** 2

        # orientation_covariance is row-major 3×3; index [8] = yaw variance.
        #
        # IMPORTANT: do NOT set index [0] to -1 here. FusionCore's
        # gnss_heading_callback rejects the whole message when
        # orientation_covariance[0] < 0 ("invalid orientation covariance"),
        # so a -1 sentinel silently drops every heading update. The heading
        # callback only reads index [8] (yaw); roll/pitch are unused. Mark
        # roll/pitch as "unknown" with a large positive variance so index [0]
        # stays >= 0 and the validity gate passes.
        _UNKNOWN = 1.0e6
        out.orientation_covariance = [
            _UNKNOWN, 0.0,      0.0,
            0.0,      _UNKNOWN, 0.0,
            0.0,      0.0,      variance,
        ]

        # Angular velocity and linear acceleration unused (heading only)
        out.angular_velocity_covariance[0]    = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        self._pub.publish(out)
        self._pub_count += 1

    def _log_stats(self) -> None:
        self.get_logger().info(
            f'heading shim: published={self._pub_count}  '
            f'rejected={self._reject_count} (invalid flags or short baseline)')
        self._pub_count    = 0
        self._reject_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = RelPosnedHeadingShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
