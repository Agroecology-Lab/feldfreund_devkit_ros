"""
recon_dem_logger.py
────────────────────
Logs RTK-fixed position (lat, lon, alt) at fixed distance intervals during a
recon drive, for later interpolation into a DEM (see devkit_ui/terrain_mask.py
and repo issue #110).

Off by default — this is a recon-mode tool, not something that should log on
every normal mission. Enable per-recon-drive via the `recon_logging.enabled`
parameter.

Distance gating is computed from /odom (metric, local frame), not from
lat/lon deltas — cheaper and avoids doing geodesy just to decide whether to
log. Only points with an RTK-fixed status are kept (STATUS_GBAS_FIX, matching
the mapping rtk_navsatfix_shim.py already applies to /gnss/fix) — DEM quality
degrades fast on float/DGPS-quality altitude.

Subscriptions
─────────────
  /gnss/fix   sensor_msgs/NavSatFix   RTK-corrected position (see
                                       rtk_navsatfix_shim.py for status codes)
  /odom       nav_msgs/Odometry       local x,y for distance-interval gating

Output
──────
  CSV at recon_logging.output_path (default /workspace/maps/recon_logs/
  recon.csv, so it survives container recreation like the rest of maps/ —
  the old ~/recon_logs default lived outside every bind mount in
  docker-compose.yml and was lost on every restart), columns:
  stamp,lat,lon,alt,x,y
"""

import csv
import math
from pathlib import Path

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus


class ReconDEMLogger:
    """Log RTK-fixed points at fixed distance intervals for DEM building."""

    def __init__(self, node: Node):
        self.log = node.get_logger()
        self._node = node

        node.declare_parameter('recon_logging.enabled', False)
        node.declare_parameter('recon_logging.min_interval_m', 0.5)
        node.declare_parameter('recon_logging.max_fix_age_s', 1.0)
        node.declare_parameter('recon_logging.output_path',
                                '/workspace/maps/recon_logs/recon.csv')

        self._enabled = node.get_parameter('recon_logging.enabled').value
        self._min_interval_m = float(
            node.get_parameter('recon_logging.min_interval_m').value)

        # Validate min_interval_m: a zero/negative value bypasses the
        # distance gate (logs every eligible odometry message); a
        # non-finite value blocks all writes after the first row.
        if not math.isfinite(self._min_interval_m) or self._min_interval_m <= 0:
            raise ValueError(
                f'recon_logging.min_interval_m must be positive and finite, '
                f'got {self._min_interval_m}')

        self._max_fix_age_s = float(
            node.get_parameter('recon_logging.max_fix_age_s').value)

        # Validate max_fix_age_s: reject negative and non-finite values before
        # the age-check logic uses them (see line 107 timestamp comparison).
        if not math.isfinite(self._max_fix_age_s) or self._max_fix_age_s < 0:
            raise ValueError(
                f'recon_logging.max_fix_age_s must be non-negative and finite, '
                f'got {self._max_fix_age_s}')

        self._output_path = Path(
            node.get_parameter('recon_logging.output_path').value)

        self._last_logged_xy: tuple[float, float] | None = None
        self._latest_fix: NavSatFix | None = None
        self._csv_file = None
        self._csv_writer = None

        if not self._enabled:
            self.log.info('ReconDEMLogger disabled (recon_logging.enabled=false)')
            return

        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self._output_path.exists()
        # pylint: disable=consider-using-with
        # Long-lived handle: released in close(), called during node shutdown.
        self._csv_file = open(self._output_path, 'a', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)
        if write_header:
            self._csv_writer.writerow(['stamp', 'lat', 'lon', 'alt', 'x', 'y'])
        self.log.info(f'ReconDEMLogger active, writing to {self._output_path} '
                       f'every {self._min_interval_m}m')

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(NavSatFix, '/gnss/fix', self._store_fix, sensor_qos)
        node.create_subscription(Odometry, '/odom', self._handle_odom, sensor_qos)

    def _store_fix(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _handle_odom(self, msg: Odometry) -> None:
        if not self._enabled or self._csv_writer is None:
            return
        if self._latest_fix is None:
            return
        if self._latest_fix.status.status < NavSatStatus.STATUS_GBAS_FIX:
            # Not RTK-fixed (see rtk_navsatfix_shim.py) — altitude isn't
            # trustworthy enough to feed a DEM.
            return

        # Reject a stale fix before pairing it with the current pose: if
        # /gnss/fix stops publishing or drops out of RTK, _latest_fix would
        # otherwise sit unchanged while x,y keeps moving, writing positions
        # that don't match the recorded pose and corrupting the DEM.
        fix_stamp = (self._latest_fix.header.stamp.sec
                     + self._latest_fix.header.stamp.nanosec * 1e-9)
        odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if abs(odom_stamp - fix_stamp) > self._max_fix_age_s:
            self.log.warning(
                f'Skipping DEM point: GNSS fix is {odom_stamp - fix_stamp:.1f}s old',
                throttle_duration_sec=5.0)
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_logged_xy is not None:
            dx = x - self._last_logged_xy[0]
            dy = y - self._last_logged_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self._min_interval_m:
                return

        self._csv_writer.writerow([
            fix_stamp,
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            self._latest_fix.altitude,
            x, y,
        ])
        self._csv_file.flush()
        self._last_logged_xy = (x, y)

    def close(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
