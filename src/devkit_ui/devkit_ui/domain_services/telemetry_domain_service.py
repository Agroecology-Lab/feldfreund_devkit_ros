from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from rclpy.qos import (
    DurabilityPolicy,
    Duration,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Bool

from devkit_ui.ros_gateway import RosGateway, TransformUnavailable

_SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

_ODOM_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

SAFETY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)

# s — map->base_link older than this: don't draw it
TF_STALENESS_LIMIT = 2.0

# Position covariance (diagonal xx) threshold below which a /fusion/odom message is trusted
# enough to take over from ground truth. fusioncore publishes early, low-confidence estimates
# before heading validates / lever arm resolves (e.g. covariance still huge, origin at 0,0) —
# latching onto the FIRST message unconditionally froze the UI marker on a garbage pose
# forever, since /odom stops updating latest_odom the instant any /fusion/odom message
# arrives. Now we keep tracking /odom until fusion's own reported covariance says it's
# actually trustworthy.
#
# Covariance alone is not enough: a UKF anchored to a degenerate GNSS origin (e.g. the world
# had no <spherical_coordinates>, so every fix was frozen at lat=0/lon=0) can report LOW
# covariance while dead reckoning off pure IMU+encoder with zero real GNSS correction —
# confidently wrong, not uncertain. Low covariance only means "the filter is internally
# consistent", not "the filter is right". So also require a real GNSS fix within the last few
# seconds before trusting /fusion/odom at all. This is belt-and-suspenders on top of fixing
# the actual root cause (missing spherical_coordinates in the generated world): it stops the
# UI from silently re-trusting a confidently-wrong fusion pose if that world-georeference
# patch ever regresses again.
FUSION_COV_TRUST_THRESHOLD = 1.0  # m^2 — matches fusioncore_sim.yaml's loosened floor
FUSION_GNSS_STALENESS_LIMIT = 5.0  # s — real /gnss/fix must be this fresh

# A real fix seen within this many seconds suppresses the sim shim's fallback fix.
FAKE_GPS_YIELD_WINDOW = 20.0  # s

# Sentinel marking our own synthetic fixes. status.service is uint16 and real receivers only
# set the low bits (GPS=1/GLONASS=2/COMPASS=4/GALILEO=8, max 15), so a high value is
# unambiguous and assignable.
FAKE_GPS_SENTINEL = 0xF000


class TelemetryDomainService:
    """Own the robot's sensor subscriptions and the trust rules for the values derived from them.

    Responsibilities:
    - Subscribe to GNSS, battery, bumper, e-stop and odometry topics and keep the latest values
    - Decide which odometry source drives the UI marker (fusion once trustworthy, else wheel odom)
    - In simulation, publish a fallback GPS fix on a dedicated topic and use it only until a
      real fix arrives
    - Look up the robot's pose in the map frame through TF

    Freshness here is a real-world-elapsed-seconds concept regardless of simulator state, so
    all of it uses the gateway's wall clock rather than the node clock. The node runs with
    use_sim_time=True in sim mode, and before Gazebo publishes /clock a sim-time clock is
    frozen at 0 — a timer driven by it never fires and "time since the last real fix"
    comparisons all read as "just happened". That silently defeated the fake-GPS shim, so the
    topo-map save path always failed with "no GPS fix yet" until Gazebo was started.
    """

    FAKE_GPS_TOPIC = '/gnss/fix_sim_shim'

    def __init__(self, ros: RosGateway, is_sim: bool,
                 fake_gps_datum: tuple[float, float], fake_gps_alt: float = 40.0) -> None:
        """Subscribe to the telemetry topics and, when is_sim is set, start the fake-GPS shim.

        Parameters:
            ros: ROS gateway for subscriptions, timers, TF and time.
            is_sim: Whether the robot is simulated. The authoritative signal is plumbed from
                manage.py's is_sim through devkit.launch.py -> ui.launch.py, so the shim is
                never started on hardware.
            fake_gps_datum: (latitude, longitude) the shim publishes. It matches the leaflet
                centre / F2C fallback used elsewhere in the UI.
            fake_gps_alt: Altitude in metres the shim publishes.
        """
        self._ros = ros
        self._is_sim = is_sim
        self._fake_lat, self._fake_lon = fake_gps_datum
        self._fake_alt = fake_gps_alt

        self.latest_odom: Odometry | None = None
        self.latest_gps: NavSatFix | None = None
        self.latest_battery: BatteryState | None = None

        self.bumper_front_top_active = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active = False
        self.estop_front_active = False
        self.estop_back_active = False

        self._last_real_gps_t = 0.0
        self._fusion_odom_seen = False
        self._pose_fail_log_t = 0.0
        self._fake_gps_pub = None

        ros.create_subscription(NavSatFix, '/gnss/fix', self._on_gps, _SENSOR_QOS)

        # Sim GPS shim: saving a topo map hard-requires a finite, non-zero fix to anchor nodes
        # to a datum, and at cold start nothing has published one yet. Gazebo's real navsat
        # sensor IS bridged onto /gnss/fix (ros_gz_bridge.yaml) — this shim used to publish
        # onto that SAME topic and rely on a discovery-time backoff to yield to the real
        # bridge. That was racy: DDS discovery has latency, so a bridge that starts publishing
        # in the same window could be missed, letting one fake fix at the hardcoded datum
        # reach fusioncore. That datum is ~53m from a real field's actual datum — a jump big
        # enough to trip fusioncore's outlier gate and anchor it on the wrong reference for
        # the rest of the run, silently rejecting every subsequent real fix. Fix: publish on a
        # dedicated topic so there is no shared-topic race at all, and only let the UI treat
        # it as a real-position fallback when no genuine /gnss/fix has arrived recently —
        # fusioncore never subscribes to this topic, so it can no longer be corrupted by the
        # shim regardless of timing.
        if is_sim:
            self._fake_gps_pub = ros.create_publisher(
                NavSatFix, self.FAKE_GPS_TOPIC, _SENSOR_QOS)
            ros.create_subscription(
                NavSatFix, self.FAKE_GPS_TOPIC, self._on_fake_gps, _SENSOR_QOS)
            # Wall-clock timer: a sim-time timer never fires before Gazebo publishes /clock,
            # which would silently disable this shim for the entire cold-start window it
            # exists to cover.
            ros.create_timer(1.0, self._publish_fake_gps, clock=ros.wall_clock())
            ros.get_logger().info(
                f'Sim mode: publishing fake fix on {self.FAKE_GPS_TOPIC} at datum '
                f'({self._fake_lat}, {self._fake_lon}) — fusioncore '
                'does not subscribe to this topic')

        ros.create_subscription(BatteryState, 'battery_state', self._on_battery, 1)
        ros.create_subscription(
            Bool, 'bumper/front_top', self._on_bumper_front_top, SAFETY_QOS)
        ros.create_subscription(
            Bool, 'bumper/front_bottom', self._on_bumper_front_bottom, SAFETY_QOS)
        ros.create_subscription(Bool, 'bumper/back', self._on_bumper_back, SAFETY_QOS)
        ros.create_subscription(Bool, 'estop/front', self._on_estop_front, SAFETY_QOS)
        ros.create_subscription(Bool, 'estop/back', self._on_estop_back, SAFETY_QOS)

        ros.create_subscription(Odometry, '/fusion/odom', self._on_fusion_odom, _ODOM_QOS)
        ros.create_subscription(Odometry, '/odom', self._on_odom_fallback, _ODOM_QOS)
        # /odometry/global is fed by a relay of /fusion/odom in sim (see sim_nav.launch.py) —
        # same trust gating applies via _on_odom_fallback's _fusion_odom_seen check, so it
        # won't overwrite a good pose with a stale/uninitialized one either.
        ros.create_subscription(Odometry, '/odometry/global', self._on_odom_fallback, _ODOM_QOS)

        # TF: robot_pose() needs the actual map->base_link transform, not a raw odom-frame
        # pose. The odom frame origin is wherever the robot started dead-reckoning (spawn
        # point in sim) — it does NOT coincide with map (0,0), so plotting raw /odom against
        # topo nodes (map frame) puts the marker off wherever it actually is, potentially
        # off-canvas entirely. The buffer gives us a real map->base_link lookup regardless of
        # whether map->odom is a static bootstrap transform (sim) or a live localisation
        # output (real hardware).
        ros.start_tf_listener()

    # ── GNSS ──────────────────────────────────────────────────────────────────

    def _on_gps(self, msg: NavSatFix) -> None:
        """Cache the latest real GNSS fix and refresh the wall-clock staleness timestamp."""
        self.latest_gps = msg
        # Anything arriving on the real /gnss/fix topic is by definition a real fix, because
        # the shim publishes elsewhere. Only refresh the timestamp for valid fixes: at least
        # STATUS_FIX, finite coordinates, and not (0,0). Leave it unchanged for invalid or
        # no-fix messages, preserving the fake-GPS fallback behaviour and the UI's last
        # usable fix.
        if (msg.status.status >= NavSatStatus.STATUS_FIX
                and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
                and not (msg.latitude == 0.0 and msg.longitude == 0.0)):
            self._last_real_gps_t = self._ros.wall_time_sec()

    def _on_fake_gps(self, msg: NavSatFix) -> None:
        """Consume the sim shim's fix as a fallback only.

        This topic is never seen by fusioncore, so it is purely for the UI's own use (e.g. the
        topo-map save path needing a finite fix at cold start before the real bridge has
        published one). Content-gated rather than topic-gated: only takes effect if no real
        fix has arrived recently, so a slow-starting real bridge doesn't leave the UI without
        any fix while it comes up.
        """
        if self._ros.wall_time_sec() - self._last_real_gps_t < FAKE_GPS_YIELD_WINDOW:
            return  # a real fix was seen recently; don't override it
        self.latest_gps = msg

    def _publish_fake_gps(self) -> None:
        """Publish a fix at the field datum (sim only — the timer isn't created on hardware)."""
        msg = NavSatFix()
        # Wall clock: the node clock is frozen at 0 before Gazebo publishes /clock, which
        # would stamp every cold-start fix identically.
        msg.header.stamp = self._ros.wall_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = FAKE_GPS_SENTINEL
        msg.latitude = self._fake_lat
        msg.longitude = self._fake_lon
        msg.altitude = self._fake_alt
        self._fake_gps_pub.publish(msg)

    # ── Odometry ──────────────────────────────────────────────────────────────

    def _on_fusion_odom(self, msg: Odometry) -> None:
        """Update the latest odometry from fusion once its covariance is trustworthy."""
        cov_xx = msg.pose.covariance[0]
        if cov_xx <= 0.0 or cov_xx > FUSION_COV_TRUST_THRESHOLD:
            return  # not trustworthy yet — let /odom keep driving the marker
        # Same wall clock the real-fix timestamp is recorded on.
        if self._ros.wall_time_sec() - self._last_real_gps_t > FUSION_GNSS_STALENESS_LIMIT:
            return  # low covariance but no recent real GNSS correction —
                    # confidently wrong, not confidently right
        self._fusion_odom_seen = True
        self.latest_odom = msg

    def _on_odom_fallback(self, msg: Odometry) -> None:
        """Use wheel odometry whenever fusion odometry has not yet been trusted.

        The original guard (only when nothing had arrived yet) froze the value after the first
        message, giving a stale pose for every subsequent drop/save. We instead update
        continuously until fusion odometry has been accepted.
        """
        if not self._fusion_odom_seen:
            self.latest_odom = msg

    # ── Battery and safety inputs ─────────────────────────────────────────────

    def _on_battery(self, msg: BatteryState) -> None:
        """Cache the latest battery state."""
        self.latest_battery = msg

    def _on_bumper_front_top(self, msg: Bool) -> None:
        """Record whether the front-top bumper is pressed."""
        self.bumper_front_top_active = msg.data

    def _on_bumper_front_bottom(self, msg: Bool) -> None:
        """Record whether the front-bottom bumper is pressed."""
        self.bumper_front_bottom_active = msg.data

    def _on_bumper_back(self, msg: Bool) -> None:
        """Record whether the rear bumper is pressed."""
        self.bumper_back_active = msg.data

    def _on_estop_front(self, msg: Bool) -> None:
        """Record whether the front hardware e-stop is active."""
        self.estop_front_active = msg.data

    def _on_estop_back(self, msg: Bool) -> None:
        """Record whether the rear hardware e-stop is active."""
        self.estop_back_active = msg.data

    # ── Pose ──────────────────────────────────────────────────────────────────

    def robot_pose(self) -> tuple[float, float, float] | None:
        """Return (x, y, yaw) of the robot in the map frame, or None if unavailable.

        This is a real map->base_link TF lookup, not raw /odom: odom's origin is the robot's
        dead-reckoning start point (spawn in sim), not map (0,0), so using it raw plots the
        marker off by the full map->odom offset.

        Liveness and staleness are both checked against the TF result itself (not
        latest_odom) so the marker tracks the thing being drawn: if /odom dies but TF is
        still fresh, keep showing it; if TF stalls, blank it even if /odom is still ticking.
        """
        # Failures are logged at most about once a second, so a persistent failure doesn't
        # flood the log across many UI refresh ticks. This distinguishes "TF lookup failed"
        # from "TF stale" when the marker disappears on navigation start.
        now_node = self._ros.now_wall_sec()
        can_log = (now_node - self._pose_fail_log_t) > 1.0

        try:
            transform = self._ros.lookup_transform('map', 'base_link')
        except TransformUnavailable as exc:
            if can_log:
                self._pose_fail_log_t = now_node
                self._ros.get_logger().warn(
                    f'robot_pose: TF lookup map->base_link failed ({exc})')
            return None
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        if now_node - stamp > TF_STALENESS_LIMIT:
            if can_log:
                self._pose_fail_log_t = now_node
                self._ros.get_logger().warn(
                    f'robot_pose: TF map->base_link stale by '
                    f'{now_node - stamp:.2f}s (limit {TF_STALENESS_LIMIT}s)')
            return None
        pos, quat = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (quat.w * quat.z + quat.x * quat.y),
                         1 - 2 * (quat.y * quat.y + quat.z * quat.z))
        return (pos.x, pos.y, yaw)
