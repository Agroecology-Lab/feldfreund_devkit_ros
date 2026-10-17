"""Test that RosGateway converts QosSpec values to rclpy QoS profiles and reads node parameters."""
import enum
import importlib
import sys
import unittest
from dataclasses import replace
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import devkit_ui

# Other tests import these modules under their own stubs and leave stale copies behind. Import fresh
# copies together so QosSpec in the gateway is the class the qos constants are built from.
with (
    patch.object(devkit_ui, 'qos', create=True),
    patch.object(devkit_ui, 'ros_gateway', create=True),
    patch.dict(sys.modules, {'rclpy': ModuleType('rclpy'),
                             'rclpy.node': SimpleNamespace(Node=type('Node', (), {}))}),
):
    sys.modules.pop('devkit_ui.qos', None)
    sys.modules.pop('devkit_ui.ros_gateway', None)
    qos = importlib.import_module('devkit_ui.qos')
    _gateway_module = importlib.import_module('devkit_ui.ros_gateway')
RosGateway = _gateway_module.RosGateway
to_qos_profile = _gateway_module.to_qos_profile


class _Reliability(enum.Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class _Durability(enum.Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class _History(enum.Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class _Liveliness(enum.Enum):
    AUTOMATIC = 1
    MANUAL_BY_TOPIC = 3


def _rclpy_qos_stub() -> ModuleType:
    """Build an rclpy.qos stand-in whose QoSProfile records its keyword arguments."""
    module = ModuleType('rclpy.qos')
    module.QoSProfile = SimpleNamespace
    module.Duration = lambda seconds: ('duration', seconds)
    module.ReliabilityPolicy = _Reliability
    module.DurabilityPolicy = _Durability
    module.HistoryPolicy = _History
    module.LivelinessPolicy = _Liveliness
    return module


class TestToQosProfile(unittest.TestCase):
    def setUp(self):
        """Provide the rclpy.qos stand-in for the duration of each test."""
        patcher = patch.dict(sys.modules, {'rclpy.qos': _rclpy_qos_stub()})
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_depth_and_existing_profiles_pass_through_unchanged(self):
        """Verify an integer depth or an rclpy profile is handed to rclpy as given."""
        profile = object()
        self.assertEqual(to_qos_profile(10), 10)
        self.assertIs(to_qos_profile(profile), profile)

    def test_sensor_spec_sets_only_depth_and_reliability(self):
        """Verify a spec with no optional fields leaves durability, history and liveliness at rclpy defaults."""
        self.assertEqual(vars(to_qos_profile(qos.SENSOR_QOS)),
                         {'depth': 1, 'reliability': _Reliability.BEST_EFFORT})

    def test_odom_spec_is_reliable_with_depth_ten(self):
        """Verify the odometry spec keeps depth 10 and reliable delivery."""
        self.assertEqual(vars(to_qos_profile(qos.ODOM_QOS)),
                         {'depth': 10, 'reliability': _Reliability.RELIABLE})

    def test_latched_spec_is_reliable_transient_local_keep_last(self):
        """Verify map and obstacle topics stay latched so a late subscriber gets the last message."""
        self.assertEqual(vars(to_qos_profile(qos.LATCHED_QOS)), {
            'depth': 1, 'reliability': _Reliability.RELIABLE,
            'durability': _Durability.TRANSIENT_LOCAL, 'history': _History.KEEP_LAST})

    def test_safety_spec_is_latched_with_automatic_liveliness_and_one_second_lease(self):
        """Verify the safety profile keeps the latched, liveliness-checked settings of the driver."""
        self.assertEqual(vars(to_qos_profile(qos.SAFETY_QOS)), {
            'depth': 1, 'reliability': _Reliability.RELIABLE,
            'durability': _Durability.TRANSIENT_LOCAL,
            'liveliness': _Liveliness.AUTOMATIC,
            'liveliness_lease_duration': ('duration', 1.0)})

    def test_optional_policies_can_be_set_independently(self):
        """Convert each alternate policy without overriding other rclpy defaults."""
        for field, value, expected in (
                ('durability', 'volatile', _Durability.VOLATILE),
                ('history', 'keep_all', _History.KEEP_ALL),
                ('liveliness', 'manual_by_topic', _Liveliness.MANUAL_BY_TOPIC)):
            with self.subTest(field=field):
                spec = replace(qos.SENSOR_QOS, **{field: value})
                self.assertEqual(vars(to_qos_profile(spec)), {
                    'depth': 1, 'reliability': _Reliability.BEST_EFFORT, field: expected})

    def test_custom_profile_combines_all_optional_policies(self):
        """Keep a zero depth for KEEP_ALL and convert all explicitly supplied settings."""
        spec = qos.QosSpec(
            depth=0, reliability='best_effort', durability='volatile', history='keep_all',
            liveliness='manual_by_topic', liveliness_lease_sec=0.25)
        self.assertEqual(vars(to_qos_profile(spec)), {
            'depth': 0, 'reliability': _Reliability.BEST_EFFORT,
            'durability': _Durability.VOLATILE, 'history': _History.KEEP_ALL,
            'liveliness': _Liveliness.MANUAL_BY_TOPIC,
            'liveliness_lease_duration': ('duration', 0.25)})

    def test_lease_preserves_zero_and_fractional_seconds_without_requiring_liveliness(self):
        """Only None omits the duration; zero is an explicit lease, measured in seconds."""
        for seconds in (None, 0.0, 0.125, 2.5):
            with self.subTest(seconds=seconds), patch.object(
                    sys.modules['rclpy.qos'], 'Duration') as duration:
                spec = replace(qos.SENSOR_QOS, liveliness_lease_sec=seconds)
                profile = to_qos_profile(spec)
                expected = {'depth': 1, 'reliability': _Reliability.BEST_EFFORT}
                if seconds is None:
                    duration.assert_not_called()
                else:
                    duration.assert_called_once_with(seconds=seconds)
                    expected['liveliness_lease_duration'] = duration.return_value
                self.assertEqual(vars(profile), expected)

    def test_unknown_policy_names_raise_before_constructing_a_profile(self):
        """Invalid policy names must remain visible instead of silently selecting defaults."""
        for field in ('reliability', 'durability', 'history', 'liveliness'):
            with self.subTest(field=field), patch.object(
                    sys.modules['rclpy.qos'], 'QoSProfile') as factory:
                spec = replace(qos.SENSOR_QOS, **{field: 'unknown_policy'})
                with self.assertRaises(KeyError) as raised:
                    to_qos_profile(spec)
                self.assertEqual(raised.exception.args, ('unknown_policy',))
                factory.assert_not_called()

    def test_duration_and_profile_constructor_errors_propagate_unchanged(self):
        """The adapter must preserve rclpy validation errors for its caller."""
        for constructor in ('Duration', 'QoSProfile'):
            failure = ValueError(f'{constructor} rejected the settings')
            with self.subTest(constructor=constructor), patch.object(
                    sys.modules['rclpy.qos'], constructor, side_effect=failure):
                with self.assertRaises(ValueError) as raised:
                    to_qos_profile(qos.SAFETY_QOS)
                self.assertIs(raised.exception, failure)

    def test_converted_profiles_do_not_share_mutable_state(self):
        """Changing one ROS profile must not alter another profile or the shared spec."""
        first = to_qos_profile(qos.SAFETY_QOS)
        second = to_qos_profile(qos.SAFETY_QOS)
        self.assertIsNot(first, second)
        first.depth = 42
        self.assertEqual(second.depth, 1)
        self.assertEqual(qos.SAFETY_QOS.depth, 1)


class TestGatewayCreation(unittest.TestCase):
    def setUp(self):
        """Wrap a mock node and provide the rclpy.qos stand-in."""
        patcher = patch.dict(sys.modules, {'rclpy.qos': _rclpy_qos_stub()})
        patcher.start()
        self.addCleanup(patcher.stop)
        self.node = Mock()
        self.gateway = RosGateway(self.node)

    def test_publisher_receives_converted_profile(self):
        """Verify a spec given to create_publisher reaches the node as a profile."""
        result = self.gateway.create_publisher('Bool', 'estop/soft', qos.SAFETY_QOS)
        message_type, topic, profile = self.node.create_publisher.call_args.args
        self.assertEqual((message_type, topic), ('Bool', 'estop/soft'))
        self.assertEqual(profile.liveliness, _Liveliness.AUTOMATIC)
        self.assertIs(result, self.node.create_publisher.return_value)
        self.node.create_publisher.assert_called_once()

    def test_subscription_receives_converted_profile_and_callback(self):
        """Verify a spec given to create_subscription reaches the node as a profile, with the callback."""
        callback = Mock()
        result = self.gateway.create_subscription(
            'String', '/current_node', callback, qos.SENSOR_QOS)
        message_type, topic, received, profile = self.node.create_subscription.call_args.args
        self.assertEqual((message_type, topic), ('String', '/current_node'))
        self.assertIs(received, callback)
        self.assertEqual(profile.reliability, _Reliability.BEST_EFFORT)
        self.assertIs(result, self.node.create_subscription.return_value)
        self.node.create_subscription.assert_called_once()
        callback.assert_not_called()

    def test_integer_depth_reaches_node_unchanged(self):
        """Verify an integer queue depth is not wrapped."""
        self.gateway.create_publisher('Twist', 'cmd_vel', 1)
        self.assertEqual(self.node.create_publisher.call_args.args[2], 1)

    def test_depths_and_existing_profiles_work_without_importing_rclpy_qos(self):
        """Both factories must pass legacy QoS inputs through even without rclpy.qos."""
        profile = SimpleNamespace(depth=7, reliability=_Reliability.RELIABLE)
        callback = Mock()
        with patch.dict(sys.modules, {'rclpy.qos': None}):
            for supplied in (0, 1, 1000, profile):
                with self.subTest(qos=supplied):
                    self.node.reset_mock()
                    publisher = self.gateway.create_publisher('Message', '/topic', supplied)
                    subscription = self.gateway.create_subscription(
                        'Message', '/topic', callback, supplied)
                    self.node.create_publisher.assert_called_once_with(
                        'Message', '/topic', supplied)
                    self.node.create_subscription.assert_called_once_with(
                        'Message', '/topic', callback, supplied)
                    self.assertIs(self.node.create_publisher.call_args.args[-1], supplied)
                    self.assertIs(self.node.create_subscription.call_args.args[-1], supplied)
                    self.assertIs(publisher, self.node.create_publisher.return_value)
                    self.assertIs(subscription, self.node.create_subscription.return_value)
        callback.assert_not_called()

    def test_invalid_specs_never_create_ros_resources(self):
        """A conversion failure must reach the caller before either node factory runs."""
        callback = Mock()
        for method, args in (
                (self.gateway.create_publisher, ('Message', '/topic')),
                (self.gateway.create_subscription, ('Message', '/topic', callback))):
            for field in ('reliability', 'durability', 'history', 'liveliness'):
                with self.subTest(method=method.__name__, field=field):
                    spec = replace(qos.SENSOR_QOS, **{field: 'unknown_policy'})
                    with self.assertRaises(KeyError):
                        method(*args, spec)
        self.node.create_publisher.assert_not_called()
        self.node.create_subscription.assert_not_called()
        callback.assert_not_called()

    def test_missing_rclpy_qos_propagates_before_creating_ros_resources(self):
        """Lazy import failures for specs must not be mistaken for usable profiles."""
        callback = Mock()
        with patch.dict(sys.modules, {'rclpy.qos': None}):
            with self.assertRaises(ModuleNotFoundError):
                self.gateway.create_publisher('Message', '/topic', qos.SENSOR_QOS)
            with self.assertRaises(ModuleNotFoundError):
                self.gateway.create_subscription('Message', '/topic', callback, qos.SENSOR_QOS)
        self.node.create_publisher.assert_not_called()
        self.node.create_subscription.assert_not_called()
        callback.assert_not_called()

    def test_node_creation_errors_propagate_without_retrying(self):
        """Node failures preserve the original exception and never invoke the callback."""
        callback = Mock()
        for name, args in (
                ('create_publisher', ('Message', '/topic')),
                ('create_subscription', ('Message', '/topic', callback))):
            with self.subTest(method=name):
                self.node.reset_mock()
                failure = RuntimeError('node has shut down')
                factory = getattr(self.node, name)
                factory.side_effect = failure
                with self.assertRaises(RuntimeError) as raised:
                    getattr(self.gateway, name)(*args, qos.SAFETY_QOS)
                self.assertIs(raised.exception, failure)
                factory.assert_called_once()
                self.assertEqual(
                    factory.call_args.args[-1].liveliness_lease_duration, ('duration', 1.0))
        callback.assert_not_called()


if __name__ == '__main__':
    unittest.main()
