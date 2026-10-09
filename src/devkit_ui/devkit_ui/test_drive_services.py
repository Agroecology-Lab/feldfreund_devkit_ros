"""Drive commands and telemetry presentation without a running ROS graph."""
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call, patch

from devkit_ui.application_services.drive_application_service import DriveApplicationService
from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel


class FakeTwist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class FakeBool:
    def __init__(self):
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
    from devkit_ui.domain_services.drive_domain_service import DriveDomainService
    from devkit_ui.ros_gateway import RosGateway


class TestDriveServices(unittest.TestCase):
    def setUp(self):
        self.ros = Mock(spec=RosGateway)
        self.velocity_publisher = Mock()
        self.estop_publisher = Mock()
        self.ros.create_publisher.side_effect = [self.velocity_publisher, self.estop_publisher]
        self.domain = DriveDomainService(self.ros)
        self.app = DriveApplicationService(self.domain)
        self.run = RunViewModel(self.app)
        self.global_vm = GlobalViewModel(self.app)

    def test_commands_use_separate_topics(self):
        self.assertEqual(self.ros.create_publisher.call_args_list, [
            call(FakeTwist, 'cmd_vel', 1), call(FakeBool, 'estop/soft', 1),
        ])

    def test_joystick_preserves_linear_speed_and_reverses_turn_direction(self):
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
        self.run.move_joystick(0.8, 0.6)
        self.run.stop_joystick()

        messages = [entry.args[0] for entry in self.velocity_publisher.publish.call_args_list]
        self.assertEqual(len(messages), 2)
        self.assertIsNot(messages[0], messages[1])
        self.assertEqual((messages[0].linear.x, messages[0].angular.z), (0.8, -0.6))
        self.assertEqual((messages[1].linear.x, messages[1].angular.z), (0.0, 0.0))

    def test_estop_toggles_both_directions_and_keeps_messages_independent(self):
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
        self.estop_publisher.publish.side_effect = RuntimeError('publisher down')
        for current in (False, True):
            with self.subTest(current=current):
                self.global_vm.soft_estop_active = current
                with self.assertRaisesRegex(RuntimeError, 'publisher down'):
                    self.global_vm.toggle_estop()
                self.assertIs(self.global_vm.soft_estop_active, current)

    def test_velocity_publish_failure_reaches_caller(self):
        self.velocity_publisher.publish.side_effect = RuntimeError('publisher down')
        with self.assertRaisesRegex(RuntimeError, 'publisher down'):
            self.run.move_joystick(0.5, 0.1)

    def test_global_view_model_uses_service_result(self):
        app = Mock(spec=DriveApplicationService)
        app.toggle_estop.return_value = False
        vm = GlobalViewModel(app)

        vm.toggle_estop()

        app.toggle_estop.assert_called_once_with(False)
        self.assertFalse(vm.soft_estop_active)


class TestPoseLabel(unittest.TestCase):
    def setUp(self):
        self.vm = RunViewModel(Mock(spec=DriveApplicationService))
        self.odom = SimpleNamespace(pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=1.234, y=-5.678))))

    def test_formats_odometry_and_only_valid_gps_fixes(self):
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
        gps = SimpleNamespace(status=SimpleNamespace(status=0), latitude=0.0, longitude=0.0)
        self.vm.update_pose_label(self.odom, gps)
        self.assertEqual(self.vm.joystick.pose_lbl, '(1.23, -5.68)\n0.00000\n0.00000')

        self.vm.update_pose_label(self.odom, None)

        self.assertEqual(self.vm.joystick.pose_lbl, '(1.23, -5.68)')

    def test_missing_odometry_overrides_stale_label_even_with_gps(self):
        gps = SimpleNamespace(status=SimpleNamespace(status=0), latitude=51.0, longitude=-2.0)
        for fix in (gps, None):
            with self.subTest(gps=fix):
                self.vm.update_pose_label(self.odom, gps)
                self.vm.update_pose_label(None, fix)
                self.assertEqual(self.vm.joystick.pose_lbl, 'no odom')


class TestRosGateway(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.gateway = RosGateway(self.node)

    def test_publisher_forwards_arguments_and_returns_handle(self):
        for qos in (1, object()):
            with self.subTest(qos=qos):
                self.node.reset_mock()
                result = self.gateway.create_publisher(FakeTwist, 'cmd_vel', qos)
                self.node.create_publisher.assert_called_once_with(FakeTwist, 'cmd_vel', qos)
                self.assertIs(result, self.node.create_publisher.return_value)

    def test_subscription_preserves_callback_and_returns_handle(self):
        callback, qos = Mock(), object()
        result = self.gateway.create_subscription(FakeBool, 'estop/soft', callback, qos)

        self.node.create_subscription.assert_called_once_with(FakeBool, 'estop/soft', callback, qos)
        self.assertIs(result, self.node.create_subscription.return_value)

    def test_logger_and_clock_are_returned_unchanged(self):
        self.assertIs(self.gateway.get_logger(), self.node.get_logger.return_value)
        self.assertIs(self.gateway.get_clock(), self.node.get_clock.return_value)
        self.node.get_logger.assert_called_once_with()
        self.node.get_clock.assert_called_once_with()

    def test_time_reads_clock_on_every_call_and_converts_fractional_seconds(self):
        self.node.get_clock.return_value.now.side_effect = [
            SimpleNamespace(nanoseconds=value) for value in (0, 1_250_000_000, 2_750_000_000)
        ]
        self.assertEqual([self.gateway.now_wall_sec() for _ in range(3)], [0.0, 1.25, 2.75])


if __name__ == '__main__':
    unittest.main()
