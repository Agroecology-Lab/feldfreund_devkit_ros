#!/usr/bin/env python3
"""
rtk_navsatfix_shim.py
─────────────────────
Republishes /rover/fix as /gnss/fix with NavSatFix.status
corrected from UBXNavPVT.carr_soln.

Problem
───────
ublox_dgnss sets NavSatFix.status = STATUS_GBAS_FIX (2) for *both* RTK FLOAT
and RTK FIXED because sensor_msgs/NavSatFix has no float/fixed distinction.
fusioncore maps STATUS_GBAS_FIX → RTK_FIXED (4), so it always believes it has
centimetre-level accuracy even when the baseline hasn't converged.

What this shim actually fixes
──────────────────────────────
UBXNavPVT.carr_soln tells us the real answer:
  0 = no RTK      → downgrade ublox_dgnss's STATUS_GBAS_FIX to the receiver's
                    actual GPS/SBAS status so fusioncore doesn't see false GBAS
  1 = RTK FLOAT   → STATUS_SBAS_FIX (1); fusioncore sees DGPS (2), not RTK_FIXED
  2 = RTK FIXED   → STATUS_GBAS_FIX (2); fusioncore correctly sees RTK_FIXED (4)

What this shim does NOT fix
────────────────────────────
fusioncore has no RTK_FLOAT (3) fix type — it only has GPS(1), DGPS(2),
RTK_FIXED(4). We map FLOAT → DGPS (closest available) rather than RTK_FIXED.
This is conservative: fusioncore will use the HP fix covariance (which reflects
float accuracy) but will not gate on RTK quality. To get fusioncore to reject
FLOAT fixes, raise gnss.min_fix_type above 2 — but that will also reject DGPS.
Full FLOAT/FIXED discrimination requires a fusioncore extension.

Cold-start race mitigation
───────────────────────────
NavSatFix HP and UBXNavPVT are both published by ublox_dgnss, but on cold start
the HP fix often arrives before the first valid PVT message.  To avoid
forwarding early fixes with stale carr_soln, the shim waits up to
PVT_TIMEOUT_S for the first PVT before publishing any fix.  After that, it
publishes immediately regardless of PVT age (the receiver is running and
carr_soln updates at 1 Hz or better).

Subscriptions
─────────────
  /rover/fix                    sensor_msgs/NavSatFix   position fixes
  /rover/ubx_nav_pvt            ublox_ubx_msgs/UBXNavPVT  carr_soln field

Publications
────────────
  /gnss/fix                     sensor_msgs/NavSatFix   corrected status
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_ubx_msgs.msg import UBXNavPVT

# carr_soln values from ZED-F9P interface description §3.15.12
_CARR_SOLN_NONE  = 0
_CARR_SOLN_FLOAT = 1
_CARR_SOLN_FIXED = 2

# How long to wait for first PVT before giving up and forwarding fixes anyway.
# After this timeout fixes are forwarded with STATUS_NO_FIX until PVT arrives,
# so fusioncore sees something rather than nothing on a slow-starting receiver.
_PVT_TIMEOUT_S = 5.0

_SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# Output QoS for /gnss/fix. MUST be RELIABLE: fusioncore subscribes to /gnss/fix
# with the default reliable profile (rclcpp::QoS(10)). A BEST_EFFORT publisher is
# QoS-incompatible with a RELIABLE subscriber, so DDS silently refuses to deliver
# any fix and fusioncore never sees a position. Publishing RELIABLE is compatible
# with both reliable and best-effort subscribers. The input subscriptions below
# stay BEST_EFFORT to match ublox_dgnss.
_PUB_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class RtkNavSatFixShim(Node):
    def __init__(self):
        super().__init__('rtk_navsatfix_shim')

        self._carr_soln: int = _CARR_SOLN_NONE
        self._pvt_received: bool = False
        self._pub_count: int = 0
        self._held_fixes: list = []   # fixes buffered while waiting for first PVT

        self._pub = self.create_publisher(NavSatFix, '/gnss/fix', _PUB_QOS)

        self.create_subscription(
            NavSatFix, '/rover/fix',
            self._fix_cb, _SENSOR_QOS)

        self.create_subscription(
            UBXNavPVT, '/rover/ubx_nav_pvt',
            self._pvt_cb, _SENSOR_QOS)

        # After PVT_TIMEOUT_S flush held fixes with NO_FIX status so fusioncore
        # isn't starved on a receiver that's slow to produce valid PVT.
        self._pvt_timeout_timer = self.create_timer(_PVT_TIMEOUT_S, self._pvt_timeout_cb)

        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f'rtk_navsatfix_shim ready — buffering fixes for up to {_PVT_TIMEOUT_S}s '
            f'awaiting first UBXNavPVT')

    def _pvt_cb(self, msg: UBXNavPVT) -> None:
        # carr_soln is a wrapped CarrSoln message-enum in this ublox_ubx_msgs
        # build, not a plain int. Storing it raw makes the == comparisons below
        # never match (corrupting the FIXED/FLOAT status mapping) and makes it
        # unhashable, which crashes _log_stats's dict lookup. Coerce to int here.
        self._carr_soln = int(msg.carr_soln)
        if not self._pvt_received:
            self._pvt_received = True
            self._pvt_timeout_timer.cancel()
            self.get_logger().info(
                'First UBXNavPVT received — flushing held fixes with carr_soln correction')
            for held in self._held_fixes:
                self._publish_corrected(held)
            self._held_fixes.clear()

    def _pvt_timeout_cb(self) -> None:
        """PVT never arrived within timeout — flush held fixes with NO_FIX status."""
        self._pvt_timeout_timer.cancel()
        if self._pvt_received:
            return
        self.get_logger().warn(
            f'No UBXNavPVT received within {_PVT_TIMEOUT_S}s — '
            f'forwarding {len(self._held_fixes)} held fix(es) as NO_FIX. '
            f'carr_soln correction inactive until PVT stream starts.')
        for held in self._held_fixes:
            # Emit with NO_FIX so fusioncore knows quality is unknown
            out = self._copy_fix(held)
            out.status.status = NavSatStatus.STATUS_NO_FIX
            self._pub.publish(out)
            self._pub_count += 1
        self._held_fixes.clear()

    def _fix_cb(self, msg: NavSatFix) -> None:
        if not self._pvt_received:
            self._held_fixes.append(msg)
            return
        self._publish_corrected(msg)

    def _publish_corrected(self, msg: NavSatFix) -> None:
        out = self._copy_fix(msg)

        if self._carr_soln == _CARR_SOLN_FIXED:
            # Centimetre-level — GBAS is appropriate
            out.status.status = NavSatStatus.STATUS_GBAS_FIX

        elif self._carr_soln == _CARR_SOLN_FLOAT:
            # Decimetre-level — map to SBAS so fusioncore uses DGPS (2) not
            # RTK_FIXED (4).  The HP fix covariance reflects actual float
            # accuracy so the UKF innovation gate will still be appropriate.
            # NOTE: fusioncore.gnss.min_fix_type=4 will reject this. Set it
            # to 2 if you want to fuse float fixes.
            out.status.status = NavSatStatus.STATUS_SBAS_FIX

        else:
            # No RTK correction — pass through receiver's own status.
            # This correctly downgrades a stale GBAS status when corrections
            # are lost mid-run.
            out.status.status = msg.status.status

        self._pub.publish(out)
        self._pub_count += 1

    @staticmethod
    def _copy_fix(msg: NavSatFix) -> NavSatFix:
        out = NavSatFix()
        out.header                   = msg.header
        out.latitude                 = msg.latitude
        out.longitude                = msg.longitude
        out.altitude                 = msg.altitude
        out.position_covariance      = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type
        out.status                   = NavSatStatus()
        out.status.service           = msg.status.service
        return out

    def _log_stats(self) -> None:
        carr_str = {
            _CARR_SOLN_NONE:  'NONE',
            _CARR_SOLN_FLOAT: 'FLOAT',
            _CARR_SOLN_FIXED: 'FIXED',
        }.get(self._carr_soln, str(self._carr_soln))
        self.get_logger().info(
            f'rtk_navsatfix_shim: published={self._pub_count}  '
            f'carr_soln={carr_str}  pvt_received={self._pvt_received}  '
            f'held={len(self._held_fixes)}')
        self._pub_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = RtkNavSatFixShim()
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
