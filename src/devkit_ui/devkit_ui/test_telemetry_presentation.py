"""Telemetry state remains live through the application, view model and node bridges."""
# pylint: disable=exec-used,protected-access
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from devkit_ui import test_telemetry_services as fixtures
from devkit_ui.application_services.telemetry_application_service import TelemetryApplicationService
from devkit_ui.view_models.telemetry_view_model import TelemetryViewModel


class TestTelemetryPresentation(unittest.TestCase):
    def setUp(self):
        """Drive all real telemetry layers through their registered sensor callbacks."""
        self.ros = fixtures.FakeRos()
        self.domain = fixtures.TelemetryDomainService(
            self.ros, is_sim=False, fake_gps_datum=fixtures.DATUM)
        self.app = TelemetryApplicationService(self.domain)
        self.vm = TelemetryViewModel(self.app)
        self.sub = self.ros.subscriptions

    def test_each_safety_input_is_live_and_independent_through_both_layers(self):
        """Every bumper/e-stop forwards activation and release without caching or cross-talk."""
        flags = {
            'bumper/front_top': 'bumper_front_top_active',
            'bumper/front_bottom': 'bumper_front_bottom_active',
            'bumper/back': 'bumper_back_active',
            'estop/front': 'estop_front_active',
            'estop/back': 'estop_back_active',
        }
        for topic, attribute in flags.items():
            with self.subTest(topic=topic):
                for active in (True, False):
                    self.sub[topic](SimpleNamespace(data=active))
                    for layer in (self.app, self.vm):
                        self.assertEqual(
                            {name: getattr(layer, name) for name in flags.values()},
                            {name: active if name == attribute else False
                             for name in flags.values()},
                        )

    def test_sensor_properties_follow_replacement_and_clearing(self):
        """Consumers see current message identities, including a return to missing data."""
        for gps, reading in (
            (None, None), (fixtures.fix(), fixtures.odom()),
            (fixtures.fix(lat=52.0), fixtures.odom()), (None, None),
        ):
            with self.subTest(gps=gps, reading=reading):
                if gps is None:
                    self.domain.latest_gps = None
                else:
                    self.sub['/gnss/fix'](gps)
                self.sub['/odom'](reading)
                self.assertIs(self.app.latest_gps, gps)
                self.assertIs(self.vm.gps, gps)
                self.assertIs(self.app.latest_odom, reading)
                self.assertIs(self.vm.odom, reading)

    def test_velocity_refresh_handles_reverse_stop_loss_and_recovery(self):
        """Zero readings replace motion; missing odometry holds the previous display."""
        for velocity in ((-0.75, 0.25), (0.0, 0.0), (0.125, -0.5)):
            with self.subTest(velocity=velocity):
                self.sub['/odom'](fixtures.odom(linear=velocity[0], angular=velocity[1]))
                self.assertEqual(self.app.measured_velocity(), velocity)
                self.vm.refresh()
                self.assertEqual((self.vm.linear_velocity, self.vm.angular_velocity), velocity)
                self.domain.latest_odom = None
                self.assertIsNone(self.app.measured_velocity())
                self.vm.refresh()
                self.assertEqual((self.vm.linear_velocity, self.vm.angular_velocity), velocity)

    def test_battery_empty_full_and_missing_readings_refresh_without_caching(self):
        """Battery formatting preserves valid zero values and resets the missing-data label."""
        self.assertIsNone(self.app.latest_battery)
        for percentage, voltage, expected in (
            (0.0, 0.0, '0.0%  0.0 V'),
            (1.0, 25.26, '100.0%  25.3 V'),
        ):
            with self.subTest(percentage=percentage):
                battery = SimpleNamespace(percentage=percentage, voltage=voltage)
                self.sub['battery_state'](battery)
                self.assertIs(self.app.latest_battery, battery)
                self.assertEqual(self.vm.battery_text, expected)
        self.domain.latest_battery = None
        self.assertEqual(self.vm.battery_text, '—')

    def test_pose_is_requested_live_and_unavailability_is_forwarded(self):
        """Neither layer caches the pose across successful and unavailable lookups."""
        self.domain.robot_pose = Mock(side_effect=[(1.0, 2.0, 0.5), None, (-2.0, 3.0, -0.5)])
        self.assertEqual(self.vm.robot_pose(), (1.0, 2.0, 0.5))
        self.assertIsNone(self.vm.robot_pose())
        self.assertEqual(self.vm.robot_pose(), (-2.0, 3.0, -0.5))
        self.assertEqual(self.domain.robot_pose.call_count, 3)

    def test_node_compatibility_properties_are_live_and_read_only(self):
        """Legacy odometry/GPS readers retain access without becoming extra state owners."""
        node = load_node_telemetry_bridge()
        node._telemetry_app_service = self.app
        self.assertIsNone(node.latest_gps)
        self.assertIsNone(node.latest_odom)
        for _ in range(2):
            gps, reading = fixtures.fix(), fixtures.odom()
            self.sub['/gnss/fix'](gps)
            self.sub['/odom'](reading)
            self.assertIs(node.latest_gps, gps)
            self.assertIs(node.latest_odom, reading)
        for name in ('latest_gps', 'latest_odom'):
            with self.subTest(property=name), self.assertRaises(AttributeError):
                setattr(node, name, None)


def load_node_telemetry_bridge():
    """Load the actual bridge properties using the repository's isolated AST harness pattern."""
    source = Path(__file__).with_name('ui_node.py')
    tree = ast.parse(source.read_text(encoding='utf-8'))
    node_class = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == 'NiceGuiNode')
    members = [node for node in node_class.body
               if isinstance(node, ast.FunctionDef) and node.name in ('latest_odom', 'latest_gps')]
    harness = ast.ClassDef(name='Harness', bases=[], keywords=[], body=members, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[harness], type_ignores=[]))
    namespace = {'Odometry': object, 'NavSatFix': object}
    exec(compile(module, source, 'exec'), namespace)
    return namespace['Harness']()


if __name__ == '__main__':
    unittest.main()
