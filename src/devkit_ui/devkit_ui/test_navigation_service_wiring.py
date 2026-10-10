"""Exercise changed NiceGuiNode methods with isolated ROS and UI boundaries."""
# pylint: disable=exec-used,no-member,protected-access,attribute-defined-outside-init
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.application_services.telemetry_application_service import (
    TelemetryApplicationService,
)
from devkit_ui.models import TopoDoc, TopoNode
from devkit_ui.test_run_ui import fake_ui
from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel
from devkit_ui.view_models.telemetry_view_model import TelemetryViewModel
from devkit_ui.view_models.topology_view_model import TopologyViewModel


def load_navigation_service_harness():
    """Load the real methods, following the existing navigation test harness pattern."""
    source = Path(__file__).with_name('ui_node.py')
    tree = ast.parse(source.read_text(encoding='utf-8'))
    node_class = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == 'NiceGuiNode')
    names = {'_nav_content', '_topo_doc', 'send_speed', 'toggle_estop'}
    members = [node for node in node_class.body
               if isinstance(node, ast.FunctionDef) and node.name in names]
    harness = ast.ClassDef(name='Harness', bases=[], keywords=[], body=members, decorator_list=[])
    namespace = {name: Mock() for name in (
        'JoystickControlCard', 'NodeMapCard', 'TrackCard', 'DropNodeCard',
        'RowDiscoveryCard', 'NavigationSidebar', 'attach_nav_card',
    )}
    namespace.update(ui=fake_ui, TopoDoc=TopoDoc)
    module = ast.fix_missing_locations(ast.Module(body=[harness], type_ignores=[]))
    exec(compile(module, source, 'exec'), namespace)
    return namespace['Harness'](), namespace


class TestNavigationServiceWiring(unittest.TestCase):
    def setUp(self):
        """Build a navigation UI harness with shared view models and synthetic telemetry."""
        fake_ui.reset()
        self.ui_on = self.enterContext(patch.object(fake_ui, 'on', create=True))
        self.node, self.namespace = load_navigation_service_harness()
        self.drive = Mock()
        self.node._global_vm = GlobalViewModel(self.drive)
        self.node._run_vm = RunViewModel(self.drive)
        self.node._topo_vm = TopologyViewModel(Mock())
        self.node._topo_vm.update_doc(TopoDoc(name='field', nodes=[
            TopoNode(name='A', x=0.0, y=0.0), TopoNode(name='B', x=2.0, y=3.0),
        ]))
        for name in ('start_track', 'stop_track', 'drop_topo_node', 'set_row_action',
                     'start_discovery', 'stop_discovery', 'send_nav_goal',
                     'cancel_nav_goal', 'confirm_delete_node', '_obstacle_mgr'):
            setattr(self.node, name, Mock())
        self.telemetry = SimpleNamespace(
            latest_gps=None,
            latest_odom=SimpleNamespace(
                pose=SimpleNamespace(
                    pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0))),
                twist=SimpleNamespace(twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.3), angular=SimpleNamespace(z=-0.4))),
            ),
            robot_pose=Mock(return_value=(1.0, 2.0, 0.5)),
        )
        self.node._telemetry_vm = TelemetryViewModel(TelemetryApplicationService(self.telemetry))
        self.node._nav_content()
        self.refresh = fake_ui.timers[0].callback
        self.sidebar = self.namespace['NavigationSidebar'].return_value

    def test_document_property_follows_view_model_replacement(self):
        """Verify the node document property follows replacement and clearing in the view model."""
        replacement = TopoDoc(name='new')
        self.node._topo_vm.update_doc(replacement)
        self.assertIs(self.node._topo_doc, replacement)
        self.node._topo_vm.update_doc(None)
        self.assertIsNone(self.node._topo_doc)

    def test_cards_receive_shared_topology_and_pose_state(self):
        """Verify cards receive the shared topology, run, and global view models."""
        self.namespace['NodeMapCard'].assert_called_once_with(
            topo_vm=self.node._topo_vm, pose_state=self.node._run_vm.node_map)
        self.namespace['JoystickControlCard'].assert_called_once_with(
            global_vm=self.node._global_vm, run_vm=self.node._run_vm)
        self.assertIs(self.namespace['DropNodeCard'].call_args.kwargs['topo_vm'],
                      self.node._topo_vm)

    def test_commands_delegate_without_overwriting_measured_velocity(self):
        """Verify drive commands delegate while measured odometry velocities stay unchanged."""
        self.node._telemetry_vm.linear_velocity = 0.2
        self.node._telemetry_vm.angular_velocity = -0.3
        self.drive.toggle_estop.return_value = True

        self.node.send_speed(0.8, 0.6)
        self.node.toggle_estop()

        self.drive.move_joystick.assert_called_once_with(0.8, 0.6)
        self.drive.toggle_estop.assert_called_once_with(False)
        self.assertTrue(self.node._global_vm.soft_estop_active)
        telemetry_vm = self.node._telemetry_vm
        self.assertEqual((telemetry_vm.linear_velocity, telemetry_vm.angular_velocity),
                         (0.2, -0.3))

    def test_refresh_uses_odometry_velocity_and_forwards_latest_pose(self):
        """Verify refresh updates telemetry without rebuilding an unchanged node list."""
        self.refresh()
        telemetry_vm = self.node._telemetry_vm
        self.assertEqual((telemetry_vm.linear_velocity, telemetry_vm.angular_velocity),
                         (0.3, -0.4))
        self.assertEqual(self.node._run_vm.joystick.pose_lbl, '(1.00, 2.00)')
        self.assertEqual(self.node._run_vm.node_map.robot_pose, (1.0, 2.0, 0.5))
        self.sidebar.render_nodes.assert_called_once()

        self.telemetry.robot_pose.return_value = (1.01, 2.01, 0.501)
        self.refresh()

        self.assertEqual(self.node._run_vm.node_map.robot_pose, (1.01, 2.01, 0.501))
        self.sidebar.render_nodes.assert_called_once()

    def test_refresh_without_map_still_updates_telemetry(self):
        """Verify telemetry updates without a map while map-specific refresh work is skipped."""
        self.node._topo_vm.update_doc(None)

        self.refresh()

        telemetry_vm = self.node._telemetry_vm
        self.assertEqual((telemetry_vm.linear_velocity, telemetry_vm.angular_velocity),
                         (0.3, -0.4))
        self.assertEqual(self.node._run_vm.joystick.pose_lbl, '(1.00, 2.00)')
        self.telemetry.robot_pose.assert_not_called()
        self.sidebar.render_nodes.assert_not_called()

    def test_losing_odometry_and_robot_pose_clears_display_state(self):
        """Verify lost telemetry clears the pose label and robot overlay state."""
        self.refresh()
        self.telemetry.latest_odom = None
        self.telemetry.robot_pose.return_value = None

        self.refresh()

        self.assertEqual(self.node._run_vm.joystick.pose_lbl, 'no odom')
        self.assertIsNone(self.node._run_vm.node_map.robot_pose)

    def test_node_click_ignores_invalid_payloads_and_selects_existing_node(self):
        """Verify node clicks select existing nodes and tolerate invalid data or a missing map."""
        self.assertEqual(self.ui_on.call_args.args[0], 'topo_node_clicked')
        callback = self.ui_on.call_args.args[1]
        self.node._topo_vm.set_selected_node('A')
        for args in (None, {}, {'node': ''}, {'node': 'missing'}):
            with self.subTest(args=args):
                callback(SimpleNamespace(args=args))
                self.assertEqual(self.node._topo_vm.selected_node, 'A')
        callback(SimpleNamespace(args={'node': 'B'}))
        self.assertEqual(self.node._topo_vm.selected_node, 'B')
        self.node._topo_vm.update_doc(None)
        callback(SimpleNamespace(args={'node': 'A'}))
        self.assertIsNone(self.node._topo_vm.selected_node)

    def test_sidebar_actions_use_latest_selection_and_separate_navigation_state(self):
        """Verify sidebar commands use the current selection and distinct navigation state."""
        callbacks = self.namespace['NavigationSidebar'].call_args.kwargs
        self.assertIs(callbacks['topo_vm'], self.node._topo_vm)
        self.assertIs(callbacks['nav_state'], self.node._run_vm.topo)
        callbacks['on_go']()
        self.node.send_nav_goal.assert_not_called()

        callbacks['on_select']('B')
        callbacks['on_go']()
        callbacks['on_delete']()
        callbacks['on_cancel']()

        self.node.send_nav_goal.assert_called_once_with('B')
        self.node.confirm_delete_node.assert_called_once_with('B')
        self.node.cancel_nav_goal.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
