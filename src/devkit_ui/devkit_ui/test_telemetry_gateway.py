"""Unit tests for the clock, timer and TF adapters introduced for telemetry."""
import sys
import types
import unittest
from unittest import mock


def stub_module(name, **attributes):
    """Create a scoped ROS import double without installing ROS on the test host."""
    module = types.ModuleType(name)
    vars(module).update(attributes)
    return module


with mock.patch.dict(sys.modules, {
    'rclpy': types.ModuleType('rclpy'),
    'rclpy.node': stub_module('rclpy.node', Node=type('Node', (), {})),
}):
    from devkit_ui import ros_gateway


class TestTelemetryGateway(unittest.TestCase):
    def setUp(self):
        """Supply strict test-owned ROS boundaries for each gateway instance."""
        self.node = mock.Mock()
        self.gateway = ros_gateway.RosGateway(self.node)
        self.clock_factory = mock.Mock()
        self.system_time = object()
        self.buffer_factory = mock.Mock()
        self.listener_factory = mock.Mock()
        self.time_factory = mock.Mock()
        self.errors = tuple(type(name, (Exception,), {}) for name in (
            'LookupException', 'ConnectivityException', 'ExtrapolationException',
        ))
        self.enterContext(mock.patch.dict(sys.modules, {
            'rclpy': types.ModuleType('rclpy'),
            'rclpy.clock': stub_module(
                'rclpy.clock', Clock=self.clock_factory,
                ClockType=types.SimpleNamespace(SYSTEM_TIME=self.system_time)),
            'rclpy.time': stub_module('rclpy.time', Time=self.time_factory),
            'tf2_ros': stub_module('tf2_ros', **{cls.__name__: cls for cls in self.errors}),
            'tf2_ros.buffer': stub_module('tf2_ros.buffer', Buffer=self.buffer_factory),
            'tf2_ros.transform_listener': stub_module(
                'tf2_ros.transform_listener', TransformListener=self.listener_factory),
        }))

    def test_system_clock_is_lazy_and_reused_per_gateway(self):
        """Requesting the wall clock constructs one system clock per wrapped node."""
        self.clock_factory.assert_not_called()
        clock = self.gateway.wall_clock()
        self.assertIs(clock, self.clock_factory.return_value)
        self.assertIs(self.gateway.wall_clock(), clock)
        self.clock_factory.assert_called_once_with(clock_type=self.system_time)
        ros_gateway.RosGateway(mock.Mock()).wall_clock()
        self.assertEqual(self.clock_factory.call_count, 2)
        self.node.get_clock.assert_not_called()

    def test_wall_time_converts_each_system_reading_without_using_sim_time(self):
        """Nanoseconds retain fractional seconds and successive calls read fresh time."""
        self.clock_factory.return_value.now.side_effect = [
            types.SimpleNamespace(nanoseconds=12_250_000_000),
            types.SimpleNamespace(nanoseconds=12_750_000_000),
        ]
        self.node.get_clock.return_value.now.return_value.nanoseconds = 0
        self.assertAlmostEqual(self.gateway.wall_time_sec(), 12.25)
        self.assertAlmostEqual(self.gateway.wall_time_sec(), 12.75)
        self.node.get_clock.assert_not_called()

    def test_timer_without_override_preserves_the_nodes_default_clock(self):
        """Omitting a clock must omit the keyword when delegating to rclpy."""
        callback = mock.Mock()
        timer = self.gateway.create_timer(0.25, callback)
        self.node.create_timer.assert_called_once_with(0.25, callback)
        self.assertIs(timer, self.node.create_timer.return_value)
        callback.assert_not_called()

    def test_timer_passes_the_explicit_clock_and_returns_the_timer_handle(self):
        """A shim timer must use the exact supplied clock."""
        callback = mock.Mock()
        clock = mock.Mock()
        timer = self.gateway.create_timer(1.0, callback, clock=clock)
        self.node.create_timer.assert_called_once_with(1.0, callback, clock=clock)
        self.assertIs(timer, self.node.create_timer.return_value)
        callback.assert_not_called()

    def test_tf_listener_starts_once_and_is_reused_by_lookups(self):
        """Explicit startup and repeated lookups share the same buffer and listener."""
        self.buffer_factory.assert_not_called()
        self.gateway.start_tf_listener()
        self.gateway.start_tf_listener()
        self.gateway.lookup_transform('map', 'base_link')
        self.buffer_factory.assert_called_once_with()
        self.listener_factory.assert_called_once_with(self.buffer_factory.return_value, self.node)

    def test_lookup_lazily_starts_tf_and_requests_latest_transform_in_frame_order(self):
        """The adapter returns the TF result and uses a zero-argument latest-time request."""
        buffer = self.buffer_factory.return_value
        first, second = object(), object()
        buffer.lookup_transform.side_effect = [first, second]
        self.assertIs(self.gateway.lookup_transform('map', 'base_link'), first)
        buffer.lookup_transform.assert_called_once_with(
            'map', 'base_link', self.time_factory.return_value)
        self.time_factory.assert_called_once_with()
        self.assertIs(self.gateway.lookup_transform('odom', 'gps'), second)
        buffer.lookup_transform.assert_called_with('odom', 'gps', self.time_factory.return_value)
        self.assertEqual(self.time_factory.call_count, 2)
        self.listener_factory.assert_called_once_with(buffer, self.node)

    def test_expected_tf_failures_preserve_the_reason_and_exception_cause(self):
        """All three recoverable TF exceptions cross the gateway as TransformUnavailable."""
        buffer = self.buffer_factory.return_value
        for error_type in self.errors:
            with self.subTest(error=error_type.__name__):
                failure = error_type('frame not ready')
                buffer.lookup_transform.side_effect = failure
                with self.assertRaises(ros_gateway.TransformUnavailable) as raised:
                    self.gateway.lookup_transform('map', 'base_link')
                self.assertEqual(str(raised.exception), f'{error_type.__name__}: frame not ready')
                self.assertIs(raised.exception.__cause__, failure)

        buffer.lookup_transform.side_effect = None
        self.assertIs(
            self.gateway.lookup_transform('map', 'base_link'), buffer.lookup_transform.return_value)
        self.buffer_factory.assert_called_once_with()

    def test_unexpected_tf_errors_propagate_unchanged(self):
        """A non-TF error must not be misclassified as missing localization."""
        failure = RuntimeError('buffer defect')
        self.buffer_factory.return_value.lookup_transform.side_effect = failure
        with self.assertRaises(RuntimeError) as raised:
            self.gateway.lookup_transform('map', 'base_link')
        self.assertIs(raised.exception, failure)

    def test_tf_buffers_and_listeners_are_independent_per_gateway(self):
        """Separate nodes must not share localization buffers or listener ownership."""
        first_buffer, second_buffer = mock.Mock(), mock.Mock()
        self.buffer_factory.side_effect = [first_buffer, second_buffer]
        second_node = mock.Mock()
        second_gateway = ros_gateway.RosGateway(second_node)
        self.assertIs(
            self.gateway.lookup_transform('map', 'base_link'),
            first_buffer.lookup_transform.return_value)
        self.assertIs(
            second_gateway.lookup_transform('odom', 'gps'),
            second_buffer.lookup_transform.return_value)
        self.listener_factory.assert_has_calls([
            mock.call(first_buffer, self.node), mock.call(second_buffer, second_node),
        ])
        self.gateway.start_tf_listener()
        second_gateway.start_tf_listener()
        self.assertEqual(self.buffer_factory.call_count, 2)
        self.assertEqual(self.listener_factory.call_count, 2)

    def test_timer_creation_failure_propagates_without_invoking_callback(self):
        """Failure to register a timer is visible to the caller without firing it early."""
        failure = RuntimeError('node has shut down')
        self.node.create_timer.side_effect = failure
        callback = mock.Mock()
        for clock in (None, object()):
            with self.subTest(clock=clock), self.assertRaises(RuntimeError) as raised:
                self.gateway.create_timer(1.0, callback, clock=clock)
            self.assertIs(raised.exception, failure)
        callback.assert_not_called()

    def test_lookup_returns_stamped_transform_without_applying_an_age_limit(self):
        """The gateway leaves freshness decisions to callers, including for old stamps."""
        transform = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=250)),
        )
        self.buffer_factory.return_value.lookup_transform.return_value = transform
        self.node.get_clock.return_value.now.return_value.nanoseconds = 100_000_000_000

        self.assertIs(self.gateway.lookup_transform('map', 'base_link'), transform)
        self.assertEqual((transform.header.stamp.sec, transform.header.stamp.nanosec), (1, 250))
        self.node.get_clock.assert_not_called()
        self.clock_factory.assert_not_called()

    def test_tf_buffer_creation_failure_propagates_and_can_be_retried(self):
        """Failed lazy construction leaves a later lookup able to initialize buffering."""
        failure = RuntimeError('buffer unavailable')
        buffer = mock.Mock()
        self.buffer_factory.side_effect = [failure, buffer]
        with self.assertRaises(RuntimeError) as raised:
            self.gateway.lookup_transform('map', 'base_link')
        self.assertIs(raised.exception, failure)
        self.listener_factory.assert_not_called()

        self.assertIs(
            self.gateway.lookup_transform('map', 'base_link'), buffer.lookup_transform.return_value,
        )
        self.listener_factory.assert_called_once_with(buffer, self.node)
        self.assertEqual(self.buffer_factory.call_count, 2)

    def test_wall_clock_creation_failure_can_be_retried(self):
        """A failed clock constructor must not leave a cached unusable clock."""
        failure = RuntimeError('clock unavailable')
        clock = object()
        self.clock_factory.side_effect = [failure, clock]
        with self.assertRaises(RuntimeError) as raised:
            self.gateway.wall_clock()
        self.assertIs(raised.exception, failure)
        self.assertIs(self.gateway.wall_clock(), clock)
        self.assertIs(self.gateway.wall_clock(), clock)
        self.assertEqual(self.clock_factory.call_count, 2)


if __name__ == '__main__':
    unittest.main()
