#!/usr/bin/env python3
"""
relposned_heading_shim.py
─────────────────────────
Converts ublox_dgnss NAV-RELPOSNED → compass_msgs/Compass for FusionCore.

Subscribes : /rover/ubx_nav_rel_pos_ned  (ublox_ubx_msgs/UBXNavRelPosNED)
Publishes  : /gnss/heading               (compass_msgs/Compass)

Only publishes when relPosValid (bit 2) AND relPosHeadingValid (bit 10) flags
are both set, and the antenna baseline passes the minimum length check.

NAV-RELPOSNED heading is True North referenced, degrees × 1e-5.
FusionCore compass_msgs/Compass bearing is radians, ENU convention.
NED → ENU: yaw_ENU = π/2 - heading_NED_rad
"""

import math
import rclpy
from rclpy.node import Node
from ublox_ubx_msgs.msg import UBXNavRelPosNED
from compass_msgs.msg import Compass

# NAV-RELPOSNED flags bitmask (UBX protocol ICD §3.18.15)
_FLAG_REL_POS_VALID     = (1 << 2)
_FLAG_HEADING_VALID     = (1 << 10)

# Minimum antenna separation — below this the heading geometry is unreliable
_MIN_BASELINE_M = 0.3


class RelPosnedHeadingShim(Node):

    def __init__(self):
        super().__init__('relposned_heading_shim')
        self._pub = self.create_publisher(Compass, '/gnss/heading', 10)
        self._sub = self.create_subscription(
            UBXNavRelPosNED, '/rover/ubx_nav_rel_pos_ned',
            self._cb, 10)
        self._pub_count    = 0
        self._reject_count = 0
        self.create_timer(10.0, self._log_stats)
        self.get_logger().info('relposned_heading_shim ready')

    def _cb(self, msg: UBXNavRelPosNED) -> None:
        flags = msg.flags

        # Both validity flags must be set before trusting the heading
        if not (flags & _FLAG_REL_POS_VALID) or not (flags & _FLAG_HEADING_VALID):
            self._reject_count += 1
            return

        # rel_pos_length is in metres in the ublox_dgnss ROS message
        if msg.rel_pos_length < _MIN_BASELINE_M:
            self._reject_count += 1
            return

        # rel_pos_heading: degrees × 1e-5, True North, NED convention
        heading_rad = math.radians(msg.rel_pos_heading * 1e-5)

        # Convert NED → ENU: yaw_ENU = π/2 − heading_NED
        bearing_enu = math.pi / 2.0 - heading_rad

        out = Compass()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.bearing         = bearing_enu

        # rel_pos_head_acc: degrees × 1e-5 accuracy estimate
        acc_rad      = math.radians(msg.rel_pos_head_acc * 1e-5)
        out.variance = acc_rad ** 2

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
