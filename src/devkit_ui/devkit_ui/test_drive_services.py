"""Drive commands and telemetry presentation without a running ROS graph."""
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call, patch

from devkit_ui.application_services.drive_application_service import DriveApplicationService
from devkit_ui.view_models import run_view_model
from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel


class FakeTwist:
    def __init__(self):
        """Initialize independent linear and angular vectors with zero components."""
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class FakeBool:
    def __init__(self):
        """Initialize a false boolean message for soft estop tests."""
        self.data = False


geometry_msgs = ModuleType('geometry_msgs.msg')
geometry_msgs.Twist = FakeTwist
std_msgs = ModuleType('std_msgs.msg')
std_msgs.Bool = FakeBool
rclpy_node = ModuleType('rclpy.node')
rclpy_node.Node = type('Node', (), {})

with patch.dict(sys.modules, {
    'geometry_msgs': ModuleType('geometry_msgs'),
    'geometry_msgs.msg': geometry_msgs,
    'std_msgs': ModuleType('std_msgs'),
    'std_msgs.msg': std_msgs,
    'rclpy': ModuleType('rclpy'),
    'rclpy.node': rclpy_node,
}):
    from devkit_ui import qos
    from devkit_ui.domain_services.drive_domain_service import DriveDomainService
    from devkit_ui.ros_gateway import RosGateway


class TestDriveServices(unittest.TestCase):
    def setUp(self):
        """Wire real drive services and view models to separate mock publishers."""
        self.ros = Mock(spec=RosGateway)
        self.velocity_publisher = Mock()
        self.estop_publisher = Mock()
        self.ros.create_publisher.side_effect = [self.velocity_publisher, self.estop_publisher]
        self.domain = DriveDomainService(self.ros)
        self.app = DriveApplicationService(self.domain)
        self.run = RunViewModel(self.app)
        self.global_vm = GlobalViewModel(self.app)
        self.clock = self.enterContext(patch(
            'devkit_ui.view_models.run_view_model.time.monotonic', return_value=1000.0))

    def test_commands_use_separate_topics(self):
        """Verify velocity and soft estop use their own message types and topics."""
        self.assertEqual(self.ros.create_publisher.call_args_list, [
            call(FakeTwist, 'cmd_vel', 1), call(FakeBool, 'estop/soft', qos.SAFETY_QOS),
        ])

    def test_soft_estop_is_latched_reliable_and_matches_the_driver_profile(self):
        """Verify the soft estop keeps the latched safety profile the driver and bumper topics use."""
        profile = qos.SAFETY_QOS
        self.assertEqual(
            (profile.depth, profile.reliability, profile.durability, profile.liveliness),
            (1, 'reliable', 'transient_local', 'automatic'))
        self.assertEqual(profile.liveliness_lease_sec, 1.0)
        self.assertIs(self.ros.create_publisher.call_args_list[1].args[2], qos.SAFETY_QOS)

    def test_joystick_preserves_linear_speed_and_reverses_turn_direction(self):
        """Verify joystick commands preserve forward speed and negate turn input."""
        for linear, angular in [(0.4, -0.7), (-0.5, 0.3), (0.0, 0.0)]:
            with self.subTest(linear=linear, angular=angular):
                self.velocity_publisher.reset_mock()
                self.run.move_joystick(linear, angular)

                self.velocity_publisher.publish.assert_called_once()
                msg = self.velocity_publisher.publish.call_args.args[0]
                self.assertEqual(vars(msg.linear), {'x': linear, 'y': 0.0, 'z': 0.0})
                self.assertEqual(vars(msg.angular), {'x': 0.0, 'y': 0.0, 'z': -angular})
                self.estop_publisher.publish.assert_not_called()

    def test_stop_publishes_zero_without_mutating_previous_command(self):
        """Verify stopping publishes a fresh zero command and preserves the earlier message."""
        self.run.move_joystick(0.8, 0.6)
        self.run.stop_joystick()

        messages = [entry.args[0] for entry in self.velocity_publisher.publish.call_args_list]
        self.assertEqual(len(messages), 2)
        self.assertIsNot(messages[0], messages[1])
        self.assertEqual((messages[0].linear.x, messages[0].angular.z), (0.8, -0.6))
        self.assertEqual((messages[1].linear.x, messages[1].angular.z), (0.0, 0.0))

    def test_estop_toggles_both_directions_and_keeps_messages_independent(self):
        """Verify successive estop toggles publish independent messages on the estop topic."""
        self.assertFalse(self.global_vm.soft_estop_active)
        self.global_vm.toggle_estop()
        self.assertTrue(self.global_vm.soft_estop_active)
        self.global_vm.toggle_estop()
        self.assertFalse(self.global_vm.soft_estop_active)

        messages = [entry.args[0] for entry in self.estop_publisher.publish.call_args_list]
        self.assertEqual([msg.data for msg in messages], [True, False])
        self.assertIsNot(messages[0], messages[1])
        self.velocity_publisher.publish.assert_not_called()

    def test_failed_estop_publish_preserves_last_confirmed_ui_state(self):
        """Verify a publish exception leaves the displayed estop state unchanged."""
        self.estop_publisher.publish.side_effect = RuntimeError('publisher down')
        for current in (False, True):
            with self.subTest(current=current):
                self.global_vm.soft_estop_active = current
                with self.assertRaisesRegex(RuntimeError, 'publisher down'):
                    self.global_vm.toggle_estop()
                self.assertIs(self.global_vm.soft_estop_active, current)

    def test_release_after_disconnect_stops_only_a_held_joystick(self):
        """Verify a dropped browser zeroes a held joystick but never overrides other cmd_vel sources."""
        self.run.release_joystick()
        self.velocity_publisher.publish.assert_not_called()

        self.run.move_joystick(0.5, 0.2)
        self.run.release_joystick()
        self.run.release_joystick()

        messages = [entry.args[0] for entry in self.velocity_publisher.publish.call_args_list]
        self.assertEqual([(m.linear.x, m.angular.z) for m in messages], [(0.5, -0.2), (0.0, 0.0)])

    def test_repeat_resends_a_fresh_joystick_command_until_it_is_released(self):
        """Verify a fresh held stick keeps publishing and a released one does not."""
        self.run.repeat_joystick()
        self.velocity_publisher.publish.assert_not_called()

        self.run.move_joystick(0.5, 0.2)
        self.run.repeat_joystick()
        self.run.repeat_joystick()
        self.run.stop_joystick()
        self.run.repeat_joystick()

        messages = [entry.args[0] for entry in self.velocity_publisher.publish.call_args_list]
        self.assertEqual([(m.linear.x, m.angular.z) for m in messages],
                         [(0.5, -0.2), (0.5, -0.2), (0.5, -0.2), (0.0, 0.0)])

    def test_repeat_stops_when_input_expires_without_a_release_event(self):
        """Verify repeats cannot renew the input lease and expiry stops only once."""
        self.run.move_joystick(0.5, 0.2)
        self.clock.return_value += run_view_model.JOYSTICK_INPUT_LEASE_S
        self.run.repeat_joystick()
        self.clock.return_value += 0.01
        self.run.repeat_joystick()
        self.run.repeat_joystick()
        self.run.release_joystick()

        messages = [entry.args[0] for entry in self.velocity_publisher.publish.call_args_list]
        self.assertEqual([(m.linear.x, m.angular.z) for m in messages],
                         [(0.5, -0.2), (0.5, -0.2), (0.0, 0.0)])

    def test_new_input_renews_the_lease_and_recovers_after_expiry(self):
        """Verify only new joystick input extends or restarts motion."""
        self.run.move_joystick(0.5, 0.2)
        self.clock.return_value += run_view_model.JOYSTICK_INPUT_LEASE_S
        self.run.move_joystick(0.3, -0.1)
        self.clock.return_value += run_view_model.JOYSTICK_INPUT_LEASE_S
        self.run.repeat_joystick()
        self.clock.return_value += 0.01
        self.run.repeat_joystick()
        self.run.move_joystick(0.2, 0.1)
        self.run.repeat_joystick()

        messages = [entry.args[0] for entry in self.velocity_publisher.publish.call_args_list]
        self.assertEqual(
            [(m.linear.x, m.angular.z) for m in messages],
            [(0.5, -0.2), (0.3, 0.1), (0.3, 0.1), (0.0, 0.0), (0.2, -0.1), (0.2, -0.1)])

    def test_velocity_publish_failure_reaches_caller(self):
        """Verify velocity publishing errors propagate through the view model."""
        self.velocity_publisher.publish.side_effect = RuntimeError('publisher down')
        with self.assertRaisesRegex(RuntimeError, 'publisher down'):
            self.run.move_joystick(0.5, 0.1)

    def test_global_view_model_uses_service_result(self):
        """Verify the view model stores the returned estop state without assuming a toggle."""
        app = Mock(spec=DriveApplicationService)
        app.toggle_estop.return_value = False
        vm = GlobalViewModel(app)

        vm.toggle_estop()

        app.toggle_estop.assert_called_once_with(False)
        self.assertFalse(vm.soft_estop_active)


class TestPoseLabel(unittest.TestCase):
    def setUp(self):
        """Create a run view model and odometry fixture with fractional coordinates."""
        self.vm = RunViewModel(Mock(spec=DriveApplicationService))
        self.odom = SimpleNamespace(pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=1.234, y=-5.678))))

    def test_formats_odometry_and_only_valid_gps_fixes(self):
        """Verify coordinate precision and inclusion of GPS only for valid fixes."""
        for status in (-1, 0, 1, 2):
            with self.subTest(status=status):
                gps = SimpleNamespace(status=SimpleNamespace(status=status),
                                      latitude=51.123456, longitude=-2.654321)
                self.vm.update_pose_label(self.odom, gps)
                expected = '(1.23, -5.68)'
                if status >= 0:
                    expected += '\n51.12346\n-2.65432'
                self.assertEqual(self.vm.joystick.pose_lbl, expected)

    def test_losing_gps_removes_stale_coordinates(self):
        """Verify losing GPS removes its coordinates while retaining odometry."""
        gps = SimpleNamespace(status=SimpleNamespace(status=0), latitude=0.0, longitude=0.0)
        self.vm.update_pose_label(self.odom, gps)
        self.assertEqual(self.vm.joystick.pose_lbl, '(1.23, -5.68)\n0.00000\n0.00000')

        self.vm.update_pose_label(self.odom, None)

        self.assertEqual(self.vm.joystick.pose_lbl, '(1.23, -5.68)')

    def test_missing_odometry_overrides_stale_label_even_with_gps(self):
        """Verify missing odometry replaces the pose label regardless of GPS availability."""
        gps = SimpleNamespace(status=SimpleNamespace(status=0), latitude=51.0, longitude=-2.0)
        for fix in (gps, None):
            with self.subTest(gps=fix):
                self.vm.update_pose_label(self.odom, gps)
                self.vm.update_pose_label(None, fix)
                self.assertEqual(self.vm.joystick.pose_lbl, 'no odom')


class TestRosGateway(unittest.TestCase):
    def setUp(self):
        """Wrap a mock ROS node in the real gateway."""
        self.node = Mock()
        self.gateway = RosGateway(self.node)

    def test_publisher_forwards_arguments_and_returns_handle(self):
        """Verify publisher creation preserves message type, topic, QoS, and returned handle."""
        for qos_profile in (1, object()):
            with self.subTest(qos=qos_profile):
                self.node.reset_mock()
                result = self.gateway.create_publisher(FakeTwist, 'cmd_vel', qos_profile)
                self.node.create_publisher.assert_called_once_with(FakeTwist, 'cmd_vel', qos_profile)
                self.assertIs(result, self.node.create_publisher.return_value)

    def test_subscription_preserves_callback_and_returns_handle(self):
        """Verify subscription creation forwards its callback and returns the node handle."""
        callback, qos_profile = Mock(), object()
        result = self.gateway.create_subscription(FakeBool, 'estop/soft', callback, qos_profile)

        self.node.create_subscription.assert_called_once_with(
            FakeBool, 'estop/soft', callback, qos_profile)
        self.assertIs(result, self.node.create_subscription.return_value)

    def test_logger_and_clock_are_returned_unchanged(self):
        """Verify logger and clock access return the underlying node objects."""
        self.assertIs(self.gateway.get_logger(), self.node.get_logger.return_value)
        self.assertIs(self.gateway.get_clock(), self.node.get_clock.return_value)
        self.node.get_logger.assert_called_once_with()
        self.node.get_clock.assert_called_once_with()

    def test_time_reads_clock_on_every_call_and_converts_fractional_seconds(self):
        """Verify each time query reads the clock and converts nanoseconds to seconds."""
        self.node.get_clock.return_value.now.side_effect = [
            SimpleNamespace(nanoseconds=value) for value in (0, 1_250_000_000, 2_750_000_000)
        ]
        self.assertEqual([self.gateway.now_wall_sec() for _ in range(3)], [0.0, 1.25, 2.75])


if __name__ == '__main__':
    unittest.main()
