"""Telemetry through its domain, application and view-model layers."""
# pylint: disable=protected-access,attribute-defined-outside-init
import math
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.application_services.telemetry_application_service import (
    TelemetryApplicationService,
)
from devkit_ui.view_models.telemetry_view_model import TelemetryViewModel

STATUS_NO_FIX = -1
STATUS_FIX = 0


class FakeNavSatFix:
    def __init__(self):
        """Create an empty fix with the nested message fields the service fills in."""
        self.header = SimpleNamespace(stamp=None, frame_id='')
        self.status = SimpleNamespace(status=STATUS_NO_FIX, service=0)
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0


def _stub_module(name, **attrs):
    module = ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


_rclpy_qos = _stub_module(
    'rclpy.qos',
    QoSProfile=SimpleNamespace,
    ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort', RELIABLE='reliable'),
    DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL='transient_local'),
    LivelinessPolicy=SimpleNamespace(AUTOMATIC='automatic'),
    Duration=lambda seconds: seconds,
)

with patch.dict(sys.modules, {
    'rclpy': ModuleType('rclpy'),
    'rclpy.node': _stub_module('rclpy.node', Node=type('Node', (), {})),
    'rclpy.qos': _rclpy_qos,
    'nav_msgs': ModuleType('nav_msgs'),
    'nav_msgs.msg': _stub_module('nav_msgs.msg', Odometry=type('Odometry', (), {})),
    'sensor_msgs': ModuleType('sensor_msgs'),
    'sensor_msgs.msg': _stub_module(
        'sensor_msgs.msg',
        BatteryState=type('BatteryState', (), {}),
        NavSatFix=FakeNavSatFix,
        NavSatStatus=SimpleNamespace(STATUS_NO_FIX=STATUS_NO_FIX, STATUS_FIX=STATUS_FIX),
    ),
    'std_msgs': ModuleType('std_msgs'),
    'std_msgs.msg': _stub_module('std_msgs.msg', Bool=type('Bool', (), {})),
}):
    from devkit_ui.domain_services import telemetry_domain_service as telemetry_module
    from devkit_ui.domain_services.telemetry_domain_service import TelemetryDomainService
    from devkit_ui.ros_gateway import TransformUnavailable

DATUM = (48.0, 3.6)


class FakeRos:
    """Records what the service wires up and lets a test drive time and TF."""

    def __init__(self):
        """Start at t=100 s on both clocks with no transform available."""
        self.subscriptions = {}
        self.publishers = {}
        self.timers = []
        self.wall = 100.0
        self.node_time = 100.0
        self.logger = Mock()
        self.tf_started = False
        self.transform = None
        self.transform_error = None
        self.clock = SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: 'wall-stamp'))

    def create_subscription(self, _msg_type, topic, callback, _qos):
        """Remember the callback registered for a topic."""
        self.subscriptions[topic] = callback

    def create_publisher(self, _msg_type, topic, _qos):
        """Return a mock publisher remembered by topic."""
        self.publishers[topic] = Mock()
        return self.publishers[topic]

    def create_timer(self, period, callback, clock=None):
        """Remember the timer and the clock that drives it."""
        self.timers.append((period, callback, clock))

    def wall_clock(self):
        """Return the fake system clock."""
        return self.clock

    def wall_time_sec(self):
        """Return the controllable real-time reading."""
        return self.wall

    def now_wall_sec(self):
        """Return the controllable node-clock reading."""
        return self.node_time

    def get_logger(self):
        """Return the shared mock logger."""
        return self.logger

    def start_tf_listener(self):
        """Record that TF buffering was requested."""
        self.tf_started = True

    def lookup_transform(self, _target, _source):
        """Return the configured transform or raise the configured failure."""
        if self.transform_error is not None:
            raise self.transform_error
        return self.transform


def fix(lat=48.0, lon=3.6, status=STATUS_FIX):
    return SimpleNamespace(
        latitude=lat, longitude=lon, status=SimpleNamespace(status=status, service=0))


def odom(cov_xx=0.1, linear=0.3, angular=-0.4):
    return SimpleNamespace(
        pose=SimpleNamespace(covariance=[cov_xx] + [0.0] * 35),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=linear), angular=SimpleNamespace(z=angular))),
    )


def transform(stamp_sec, x=1.0, y=2.0, qz=0.0, qw=1.0):
    whole = int(stamp_sec)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=whole, nanosec=int((stamp_sec - whole) * 1e9))),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=qz, w=qw)),
    )


class TelemetryDomainTestCase(unittest.TestCase):
    is_sim = False

    def setUp(self):
        """Build the domain service against a fake gateway."""
        self.ros = FakeRos()
        self.service = TelemetryDomainService(self.ros, is_sim=self.is_sim, fake_gps_datum=DATUM)
        self.sub = self.ros.subscriptions


class TestWiring(unittest.TestCase):
    def test_hardware_subscribes_without_sim_shim(self):
        """Verify hardware mode has no fake-GPS topic, publisher or timer."""
        ros = FakeRos()
        TelemetryDomainService(ros, is_sim=False, fake_gps_datum=DATUM)

        self.assertEqual(set(ros.subscriptions), {
            '/gnss/fix', 'battery_state', 'bumper/front_top', 'bumper/front_bottom',
            'bumper/back', 'estop/front', 'estop/back', '/fusion/odom', '/odom',
            '/odometry/global',
        })
        self.assertEqual(ros.publishers, {})
        self.assertEqual(ros.timers, [])
        self.assertTrue(ros.tf_started)

    def test_sim_adds_shim_on_its_own_topic_driven_by_the_wall_clock(self):
        """Verify sim mode publishes on a dedicated topic from a timer on the system clock."""
        ros = FakeRos()
        TelemetryDomainService(ros, is_sim=True, fake_gps_datum=DATUM)

        self.assertIn(TelemetryDomainService.FAKE_GPS_TOPIC, ros.subscriptions)
        self.assertEqual(list(ros.publishers), [TelemetryDomainService.FAKE_GPS_TOPIC])
        self.assertNotEqual(TelemetryDomainService.FAKE_GPS_TOPIC, '/gnss/fix')
        self.assertEqual([(period, clock) for period, _, clock in ros.timers],
                         [(1.0, ros.clock)])

    def test_safety_inputs_use_reliable_transient_local_qos(self):
        """Verify the bumper and e-stop QoS keeps the latched, reliable settings."""
        qos = telemetry_module.SAFETY_QOS
        self.assertEqual(qos.reliability, 'reliable')
        self.assertEqual(qos.durability, 'transient_local')
        self.assertEqual(qos.liveliness, 'automatic')


class TestGps(TelemetryDomainTestCase):
    def test_valid_real_fix_is_cached_and_timestamped(self):
        """Verify a valid fix is cached and records when it arrived."""
        message = fix()
        self.ros.wall = 150.0
        self.sub['/gnss/fix'](message)

        self.assertIs(self.service.latest_gps, message)
        self.assertEqual(self.service._last_real_gps_t, 150.0)

    def test_invalid_fixes_are_cached_but_do_not_count_as_fresh(self):
        """Verify no-fix, zero and non-finite fixes leave the freshness timestamp alone."""
        for message in (fix(status=STATUS_NO_FIX), fix(lat=0.0, lon=0.0),
                        fix(lat=math.nan), fix(lon=math.inf)):
            with self.subTest(lat=message.latitude, lon=message.longitude,
                              status=message.status.status):
                self.sub['/gnss/fix'](message)
                self.assertIs(self.service.latest_gps, message)
                self.assertEqual(self.service._last_real_gps_t, 0.0)


class TestFakeGps(TelemetryDomainTestCase):
    is_sim = True

    def test_shim_fix_is_used_when_no_real_fix_has_arrived(self):
        """Verify the cold-start fallback supplies a fix before any real one exists."""
        shim = fix()
        self.sub[TelemetryDomainService.FAKE_GPS_TOPIC](shim)
        self.assertIs(self.service.latest_gps, shim)

    def test_shim_fix_yields_to_a_recent_real_fix(self):
        """Verify a real fix less than the yield window old is not overwritten."""
        real = fix(lat=51.0, lon=-2.5)
        self.ros.wall = 100.0
        self.sub['/gnss/fix'](real)
        self.ros.wall = 100.0 + telemetry_module.FAKE_GPS_YIELD_WINDOW - 0.1

        self.sub[TelemetryDomainService.FAKE_GPS_TOPIC](fix())

        self.assertIs(self.service.latest_gps, real)

    def test_shim_fix_takes_over_once_the_real_fix_is_stale(self):
        """Verify the fallback applies again after the real receiver goes quiet."""
        self.sub['/gnss/fix'](fix(lat=51.0, lon=-2.5))
        self.ros.wall += telemetry_module.FAKE_GPS_YIELD_WINDOW + 0.1
        shim = fix()

        self.sub[TelemetryDomainService.FAKE_GPS_TOPIC](shim)

        self.assertIs(self.service.latest_gps, shim)

    def test_timer_publishes_the_datum_with_the_sentinel_and_a_wall_stamp(self):
        """Verify the published shim fix carries the datum, sentinel and wall-clock stamp."""
        _, publish, _ = self.ros.timers[0]
        publish()

        sent = self.ros.publishers[TelemetryDomainService.FAKE_GPS_TOPIC].publish.call_args.args[0]
        self.assertEqual((sent.latitude, sent.longitude, sent.altitude), (*DATUM, 40.0))
        self.assertEqual(sent.status.status, STATUS_FIX)
        self.assertEqual(sent.status.service, telemetry_module.FAKE_GPS_SENTINEL)
        self.assertEqual(sent.header.stamp, 'wall-stamp')
        self.assertEqual(sent.header.frame_id, 'gps')


class TestOdometry(TelemetryDomainTestCase):
    def _real_fix_now(self):
        self.sub['/gnss/fix'](fix())

    def test_wheel_odometry_drives_the_marker_until_fusion_is_trusted(self):
        """Verify /odom and /odometry/global both update the pose while fusion is untrusted."""
        first, second = odom(), odom()
        self.sub['/odom'](first)
        self.assertIs(self.service.latest_odom, first)
        self.sub['/odometry/global'](second)
        self.assertIs(self.service.latest_odom, second)

    def test_fusion_with_low_covariance_and_a_fresh_fix_takes_over(self):
        """Verify trusted fusion odometry replaces wheel odometry and blocks later fallbacks."""
        self._real_fix_now()
        fused = odom(cov_xx=0.5)
        self.sub['/fusion/odom'](fused)
        self.assertIs(self.service.latest_odom, fused)

        self.sub['/odom'](odom())
        self.sub['/odometry/global'](odom())

        self.assertIs(self.service.latest_odom, fused)

    def test_fusion_with_untrustworthy_covariance_is_ignored(self):
        """Verify zero, negative and large covariance leave wheel odometry in charge."""
        self._real_fix_now()
        wheel = odom()
        self.sub['/odom'](wheel)
        for cov in (0.0, -1.0, telemetry_module.FUSION_COV_TRUST_THRESHOLD + 0.01):
            with self.subTest(cov=cov):
                self.sub['/fusion/odom'](odom(cov_xx=cov))
                self.assertIs(self.service.latest_odom, wheel)

    def test_confident_fusion_without_a_recent_real_fix_is_ignored(self):
        """Verify low covariance alone does not win when no real GNSS fix is fresh."""
        wheel = odom()
        self.sub['/odom'](wheel)
        self._real_fix_now()
        self.ros.wall += telemetry_module.FUSION_GNSS_STALENESS_LIMIT + 0.1

        self.sub['/fusion/odom'](odom(cov_xx=0.1))

        self.assertIs(self.service.latest_odom, wheel)
        later_wheel = odom()
        self.sub['/odom'](later_wheel)
        self.assertIs(self.service.latest_odom, later_wheel)

    def test_fusion_is_ignored_when_no_real_fix_has_ever_arrived(self):
        """Verify the staleness gate also holds at cold start."""
        self.sub['/fusion/odom'](odom(cov_xx=0.1))
        self.assertIsNone(self.service.latest_odom)


class TestSensorInputs(TelemetryDomainTestCase):
    def test_battery_is_cached(self):
        """Verify the latest battery message is kept."""
        battery = SimpleNamespace(percentage=0.5, voltage=24.0)
        self.sub['battery_state'](battery)
        self.assertIs(self.service.latest_battery, battery)

    def test_bumpers_and_estops_follow_their_topics(self):
        """Verify each safety input updates only its own flag."""
        flags = {
            'bumper/front_top': 'bumper_front_top_active',
            'bumper/front_bottom': 'bumper_front_bottom_active',
            'bumper/back': 'bumper_back_active',
            'estop/front': 'estop_front_active',
            'estop/back': 'estop_back_active',
        }
        for topic, attr in flags.items():
            with self.subTest(topic=topic):
                self.sub[topic](SimpleNamespace(data=True))
                self.assertTrue(getattr(self.service, attr))
                self.assertEqual(
                    [name for name in flags.values() if getattr(self.service, name)], [attr])
                self.sub[topic](SimpleNamespace(data=False))
                self.assertFalse(getattr(self.service, attr))


class TestRobotPose(TelemetryDomainTestCase):
    def test_fresh_transform_returns_position_and_yaw(self):
        """Verify the pose comes from the map->base_link transform with yaw from the quaternion."""
        half = math.sqrt(0.5)
        self.ros.transform = transform(99.5, x=3.0, y=-1.0, qz=half, qw=half)

        x, y, yaw = self.service.robot_pose()

        self.assertEqual((x, y), (3.0, -1.0))
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_stale_transform_is_blanked_and_logged_once_a_second(self):
        """Verify a stale transform gives None and repeated failures are rate-limited."""
        self.ros.transform = transform(100.0 - telemetry_module.TF_STALENESS_LIMIT - 1.0)

        self.assertIsNone(self.service.robot_pose())
        self.assertIsNone(self.service.robot_pose())
        self.assertEqual(self.ros.logger.warn.call_count, 1)
        self.assertIn('stale', self.ros.logger.warn.call_args.args[0])

        self.ros.node_time += 1.5
        self.assertIsNone(self.service.robot_pose())
        self.assertEqual(self.ros.logger.warn.call_count, 2)

    def test_missing_transform_gives_none_and_logs_the_reason(self):
        """Verify a failed lookup is reported without raising."""
        self.ros.transform_error = TransformUnavailable('LookupException: no frame')

        self.assertIsNone(self.service.robot_pose())
        self.assertIn('LookupException', self.ros.logger.warn.call_args.args[0])

    def test_pose_recovers_after_a_failure(self):
        """Verify a later fresh transform is returned after an earlier failure."""
        self.ros.transform_error = TransformUnavailable('gone')
        self.assertIsNone(self.service.robot_pose())
        self.ros.transform_error = None
        self.ros.transform = transform(100.0)
        self.assertIsNotNone(self.service.robot_pose())


class TestApplicationServiceAndViewModel(unittest.TestCase):
    def setUp(self):
        """Wrap a stand-in domain service in the real application service and view model."""
        self.domain = SimpleNamespace(
            latest_odom=None, latest_gps=None, latest_battery=None,
            bumper_front_top_active=False, bumper_front_bottom_active=False,
            bumper_back_active=False, estop_front_active=False, estop_back_active=False,
            robot_pose=Mock(return_value=(1.0, 2.0, 0.5)),
        )
        self.app = TelemetryApplicationService(self.domain)
        self.vm = TelemetryViewModel(self.app)

    def test_measured_velocity_comes_from_odometry(self):
        """Verify velocity is None without odometry and read from it once present."""
        self.assertIsNone(self.app.measured_velocity())
        self.domain.latest_odom = odom(linear=0.3, angular=-0.4)
        self.assertEqual(self.app.measured_velocity(), (0.3, -0.4))

    def test_refresh_keeps_the_last_velocities_when_odometry_is_lost(self):
        """Verify the sliders hold their last reading rather than snapping to zero."""
        self.domain.latest_odom = odom(linear=0.3, angular=-0.4)
        self.vm.refresh()
        self.domain.latest_odom = None
        self.vm.refresh()

        self.assertEqual((self.vm.linear_velocity, self.vm.angular_velocity), (0.3, -0.4))

    def test_velocities_start_at_zero(self):
        """Verify the view model reads zero before any odometry arrives."""
        self.vm.refresh()
        self.assertEqual((self.vm.linear_velocity, self.vm.angular_velocity), (0.0, 0.0))

    def test_battery_text_formats_level_and_voltage(self):
        """Verify the battery label shows a dash without data and formatted values with it."""
        self.assertEqual(self.vm.battery_text, '—')
        self.domain.latest_battery = SimpleNamespace(percentage=0.756, voltage=24.04)
        self.assertEqual(self.vm.battery_text, '75.6%  24.0 V')

    def test_view_model_passes_readings_through_live(self):
        """Verify the view model reflects service state at read time, not a stale copy."""
        gps, reading = fix(), odom()
        self.domain.latest_gps, self.domain.latest_odom = gps, reading
        self.domain.bumper_back_active = True
        self.domain.estop_front_active = True

        self.assertIs(self.vm.gps, gps)
        self.assertIs(self.vm.odom, reading)
        self.assertTrue(self.vm.bumper_back_active)
        self.assertTrue(self.vm.estop_front_active)
        self.assertFalse(self.vm.bumper_front_top_active)
        self.assertEqual(self.vm.robot_pose(), (1.0, 2.0, 0.5))


if __name__ == '__main__':
    unittest.main()
