"""Regression cases for telemetry freshness, source selection and clock boundaries."""
import math
import unittest
from unittest.mock import Mock

from devkit_ui import test_telemetry_services as fixtures


class TestTelemetryBoundaries(unittest.TestCase):
    def setUp(self):
        """Use independent wall and node clocks and the registered sensor callbacks."""
        self.ros = fixtures.FakeRos()
        self.service = fixtures.TelemetryDomainService(
            self.ros, is_sim=True, fake_gps_datum=fixtures.DATUM)
        self.sub = self.ros.subscriptions

    def test_shim_resumes_at_twenty_seconds_and_yields_to_a_new_real_fix(self):
        """A paused simulation must not freeze the fallback's yield window."""
        self.ros.node_time = 0.0
        real, shim = fixtures.fix(lat=51.0), fixtures.fix()
        self.sub['/gnss/fix'](real)
        for now, expected in ((119.999, real), (120.0, shim)):
            with self.subTest(now=now):
                self.ros.wall = now
                self.sub['/gnss/fix_sim_shim'](shim)
                self.assertIs(self.service.latest_gps, expected)

        recovered = fixtures.fix(lat=52.0)
        self.sub['/gnss/fix'](recovered)
        self.sub['/gnss/fix_sim_shim'](shim)
        self.assertIs(self.service.latest_gps, recovered)

    def test_invalid_fixes_do_not_extend_the_last_valid_fix_lifetime(self):
        """Invalid readings cannot postpone fallback or authorize fusion after GNSS expires."""
        invalid_fixes = (
            fixtures.fix(status=fixtures.STATUS_NO_FIX), fixtures.fix(lat=0.0, lon=0.0),
            fixtures.fix(lat=math.nan), fixtures.fix(lon=math.nan),
            fixtures.fix(lat=math.inf), fixtures.fix(lon=-math.inf),
        )
        for invalid in invalid_fixes:
            with self.subTest(invalid=invalid):
                ros = fixtures.FakeRos()
                service = fixtures.TelemetryDomainService(
                    ros, is_sim=True, fake_gps_datum=fixtures.DATUM)
                callbacks = ros.subscriptions
                callbacks['/gnss/fix'](fixtures.fix())
                ros.wall = 119.0
                callbacks['/gnss/fix'](invalid)
                callbacks['/fusion/odom'](fixtures.odom())
                self.assertIsNone(service.latest_odom)
                ros.wall = 120.0
                shim = fixtures.fix()
                callbacks['/gnss/fix_sim_shim'](shim)
                self.assertIs(service.latest_gps, shim)

    def test_equator_and_prime_meridian_fixes_can_authorize_fusion(self):
        """A zero coordinate alone is valid; only the pair (0, 0) is rejected."""
        for lat, lon in ((0.0, 3.6), (48.0, 0.0)):
            with self.subTest(lat=lat, lon=lon):
                self.ros.wall += 10.0
                self.sub['/gnss/fix'](fixtures.fix(lat=lat, lon=lon, status=2))
                fused = fixtures.odom()
                self.sub['/fusion/odom'](fused)
                self.assertIs(self.service.latest_odom, fused)

    def test_fusion_accepts_covariance_and_gnss_age_at_their_limits(self):
        """Both inclusive boundaries admit fusion even when simulated time is frozen."""
        self.ros.node_time = 0.0
        self.sub['/gnss/fix'](fixtures.fix())
        self.ros.wall = 105.0
        fused = fixtures.odom(cov_xx=1.0)
        self.sub['/fusion/odom'](fused)
        self.assertIs(self.service.latest_odom, fused)

    def test_shim_never_authorizes_fusion(self):
        """Repeated synthetic fixes leave both fallback odometry sources usable."""
        for now, topic in ((100.0, '/odom'), (130.0, '/odometry/global')):
            with self.subTest(topic=topic):
                self.ros.wall = now
                self.sub['/gnss/fix_sim_shim'](fixtures.fix())
                wheel = fixtures.odom()
                self.sub[topic](wheel)
                self.sub['/fusion/odom'](fixtures.odom())
                self.assertIs(self.service.latest_odom, wheel)

    def test_rejected_fusion_does_not_latch_out_fallback(self):
        """Covariance rejection must leave subsequent wheel messages free to update state."""
        self.sub['/gnss/fix'](fixtures.fix())
        for covariance in (0.0, -0.1, 1.001, math.inf):
            with self.subTest(covariance=covariance):
                self.sub['/fusion/odom'](fixtures.odom(cov_xx=covariance))
                wheel = fixtures.odom()
                self.sub['/odom'](wheel)
                self.assertIs(self.service.latest_odom, wheel)

    def test_trusted_fusion_retains_last_reading_until_gnss_recovers(self):
        """The existing fusion latch survives rejected updates and permits recovery."""
        self.sub['/gnss/fix'](fixtures.fix())
        accepted = fixtures.odom()
        self.sub['/fusion/odom'](accepted)
        self.sub['/fusion/odom'](fixtures.odom(cov_xx=2.0))
        self.assertIs(self.service.latest_odom, accepted)
        self.ros.wall = 105.001
        self.sub['/fusion/odom'](fixtures.odom())
        for topic in ('/odom', '/odometry/global'):
            self.sub[topic](fixtures.odom())
            self.assertIs(self.service.latest_odom, accepted)

        self.sub['/gnss/fix'](fixtures.fix())
        recovered = fixtures.odom(linear=-0.8)
        self.sub['/fusion/odom'](recovered)
        self.assertIs(self.service.latest_odom, recovered)

    def test_sim_timer_uses_custom_datum_altitude_and_a_fresh_stamp_each_tick(self):
        """Publication creates independent messages and uses the live system clock."""
        ros = fixtures.FakeRos()
        ros.node_time = 0.0
        ros.clock.now = Mock()
        ros.clock.now.return_value.to_msg.side_effect = ['first-stamp', 'second-stamp']
        fixtures.TelemetryDomainService(
            ros, is_sim=True, fake_gps_datum=(-12.0, 0.0), fake_gps_alt=0.0)
        _, tick, _ = ros.timers[0]
        tick()
        tick()
        published = ros.publishers['/gnss/fix_sim_shim'].publish.call_args_list
        first, second = (call.args[0] for call in published)
        self.assertIsNot(first, second)
        self.assertEqual(first.header.stamp, 'first-stamp')
        self.assertEqual(second.header.stamp, 'second-stamp')
        self.assertEqual((second.latitude, second.longitude, second.altitude), (-12.0, 0.0, 0.0))

    def test_tf_age_uses_node_clock_and_includes_nanoseconds(self):
        """Freshness includes exactly two seconds and ignores unrelated wall-clock time."""
        self.ros.wall = 10_000.0
        self.ros.node_time = 10.5
        self.ros.transform = fixtures.transform(8.5)
        self.ros.lookup_transform = Mock(return_value=self.ros.transform)
        self.assertEqual(self.service.robot_pose(), (1.0, 2.0, 0.0))
        self.ros.lookup_transform.assert_called_once_with('map', 'base_link')
        self.ros.logger.warn.assert_not_called()
        self.ros.node_time = 10.501
        self.assertIsNone(self.service.robot_pose())

    def test_pose_uses_full_quaternion_and_does_not_require_odometry(self):
        """Roll and pitch terms contribute to yaw even before any odometry arrives."""
        self.ros.transform = fixtures.transform(100.0)
        rotation = self.ros.transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = (0.5, 0.5, 0.0, math.sqrt(0.5))
        self.assertIsNone(self.service.latest_odom)
        self.assertAlmostEqual(self.service.robot_pose()[2], math.pi / 4)

    def test_lookup_and_staleness_failures_share_the_warning_throttle(self):
        """Switching failure causes does not flood warnings, including at the one-second edge."""
        self.ros.transform_error = fixtures.TransformUnavailable('disconnected')
        self.assertIsNone(self.service.robot_pose())
        self.ros.transform_error = None
        self.ros.transform = fixtures.transform(90.0)
        self.ros.node_time = 101.0
        self.assertIsNone(self.service.robot_pose())
        self.assertEqual(self.ros.logger.warn.call_count, 1)
        self.ros.node_time = 101.001
        self.assertIsNone(self.service.robot_pose())
        self.assertEqual(self.ros.logger.warn.call_count, 2)

    def test_unexpected_lookup_failure_is_not_hidden(self):
        """Programming errors must propagate instead of appearing as unavailable telemetry."""
        failure = ValueError('invalid transform payload')
        self.ros.transform_error = failure
        with self.assertRaises(ValueError) as raised:
            self.service.robot_pose()
        self.assertIs(raised.exception, failure)
        self.ros.logger.warn.assert_not_called()


if __name__ == '__main__':
    unittest.main()
