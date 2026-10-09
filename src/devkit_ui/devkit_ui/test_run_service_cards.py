"""Unit tests for the service-backed joystick and topology cards."""
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.models import TopoDoc, TopoNode
from devkit_ui.test_run_ui import fake_ui, nicegui
from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel
from devkit_ui.view_models.topology_view_model import TopologyViewModel

with patch.dict(sys.modules, {'nicegui': nicegui}):
    from devkit_ui.pages.run import joystick_control_card, node_map_card


class TestJoystickControlCard(unittest.TestCase):
    def setUp(self):
        fake_ui.reset()
        self.enterContext(patch.object(joystick_control_card, 'ui', fake_ui))
        self.app = Mock()
        self.global_vm = GlobalViewModel(self.app)
        self.run_vm = RunViewModel(self.app)
        self.card = joystick_control_card.JoystickControlCard(self.global_vm, self.run_vm)

    def test_move_converts_event_values_and_maps_vertical_axis_to_forward_speed(self):
        joystick = fake_ui.find('joystick')

        joystick.kwargs['on_move'](SimpleNamespace(x='-0.25', y='0.75'))

        self.app.move_joystick.assert_called_once_with(0.75, -0.25)

    def test_release_sends_zero_speed(self):
        fake_ui.find('joystick').kwargs['on_end'](None)

        self.app.move_joystick.assert_called_once_with(0.0, 0.0)

    def test_estop_click_updates_service_state_label_and_color(self):
        self.assertEqual(self.card.estop_btn.text, 'E-Stop')
        self.assertEqual(self.card.estop_btn.properties, 'color=primary')
        self.app.toggle_estop.side_effect = [True, False]

        for expected, label, color in [(True, 'STOPPED', 'negative'), (False, 'E-Stop', 'primary')]:
            with self.subTest(expected=expected):
                current = self.global_vm.soft_estop_active
                self.card.estop_btn.on_click(None)
                self.app.toggle_estop.assert_called_with(current)
                self.assertIs(self.global_vm.soft_estop_active, expected)
                self.assertEqual(self.card.estop_btn.refresh_binding(), label)
                self.assertEqual(self.card.estop_btn.properties, f'color={color}')

    def test_pose_label_tracks_view_model(self):
        label = fake_ui.find('label', 'no odom')
        self.run_vm.joystick.pose_lbl = '(1.00, -2.00)'

        self.assertEqual(label.refresh_binding(), '(1.00, -2.00)')


class TestNodeMapCard(unittest.TestCase):
    def setUp(self):
        fake_ui.reset()
        self.enterContext(patch.object(node_map_card, 'ui', fake_ui))
        self.inject_click_js = self.enterContext(patch.object(node_map_card, 'inject_click_js'))
        self.build_svg = self.enterContext(patch.object(
            node_map_card, 'build_svg', wraps=node_map_card.build_svg))
        self.build_robot_svg = self.enterContext(patch.object(
            node_map_card, 'build_robot_svg', wraps=node_map_card.build_robot_svg))
        self.vm = TopologyViewModel(Mock())
        self.vm.update_doc(self.make_doc())
        self.pose = RunViewModel.NodeMap(robot_pose=(1.0, 1.0, 0.25))
        self.card = node_map_card.NodeMapCard(self.vm, self.pose)
        self.refresh = fake_ui.timers[0].callback

    @staticmethod
    def make_doc(*, end_x=4.0, connected=True):
        return TopoDoc(name='field', nodes=[
            TopoNode(name='A', x=0.0, y=0.0, edges=['B'] if connected else []),
            TopoNode(name='B', x=end_x, y=4.0),
        ])

    def test_first_tick_draws_both_layers_and_unchanged_ticks_do_no_work(self):
        self.inject_click_js.assert_called_once_with()
        self.assertEqual(fake_ui.timers[0].interval, 0.2)

        self.refresh()
        self.refresh()

        self.build_svg.assert_called_once_with(self.vm.topo_doc, None, '—')
        self.assert_robot_rendered_once(self.pose.robot_pose)
        self.assertIn('<svg', self.card._map_html.content)
        self.assertIn('<circle', self.card._robot_html.content)

    def test_selection_and_current_node_redraw_map_without_redrawing_robot(self):
        self.refresh()
        self.build_svg.reset_mock()
        self.build_robot_svg.reset_mock()

        self.vm.set_selected_node('B')
        self.refresh()
        self.build_svg.assert_called_once_with(self.vm.topo_doc, 'B', '—')
        self.build_svg.reset_mock()
        self.vm.set_current_node('A')
        self.refresh()

        self.build_svg.assert_called_once_with(self.vm.topo_doc, 'B', 'A')
        self.build_robot_svg.assert_not_called()

    def test_small_pose_noise_is_ignored_but_motion_and_pose_loss_update_overlay(self):
        self.refresh()
        self.build_svg.reset_mock()
        self.build_robot_svg.reset_mock()
        self.pose.robot_pose = (1.01, 1.01, 0.251)
        self.refresh()
        self.build_robot_svg.assert_not_called()

        self.pose.robot_pose = (1.11, 1.0, 0.27)
        self.refresh()
        self.assert_robot_rendered_once(self.pose.robot_pose)
        self.build_robot_svg.reset_mock()
        self.pose.robot_pose = None
        self.refresh()

        self.assert_robot_rendered_once(None)
        self.assertNotIn('<circle', self.card._robot_html.content)
        self.build_svg.assert_not_called()

    def test_node_addition_updates_both_layers(self):
        self.refresh()
        self.build_svg.reset_mock()
        self.build_robot_svg.reset_mock()
        self.vm.topo_doc.add_node(TopoNode(name='C', x=8.0, y=8.0))

        self.refresh()

        self.build_svg.assert_called_once()
        self.build_robot_svg.assert_called_once()
        self.assertIn('C', self.card._map_html.content)

    def test_empty_document_displays_placeholder_and_clears_robot(self):
        self.refresh()
        self.vm.update_doc(TopoDoc(name='empty'))

        self.refresh()

        self.assertIn('No map loaded', self.card._map_html.content)
        self.assertNotIn('<circle', self.card._robot_html.content)

    def test_clearing_document_displays_placeholder_and_clears_robot(self):
        self.refresh()
        self.vm.update_doc(None)

        self.refresh()

        self.assertIn('No map loaded', self.card._map_html.content)
        self.assertNotIn('<circle', self.card._robot_html.content)

    def test_same_names_with_new_positions_refresh_map_and_robot_bounds(self):
        self.refresh()
        previous_robot = self.card._robot_html.content
        self.build_svg.reset_mock()
        self.build_robot_svg.reset_mock()
        self.vm.update_doc(self.make_doc(end_x=8.0))

        self.refresh()

        self.build_svg.assert_called_once_with(self.vm.topo_doc, None, '—')
        self.assert_robot_rendered_once(self.pose.robot_pose)
        self.assertNotEqual(self.card._robot_html.content, previous_robot)

    def test_same_names_with_changed_edges_refresh_map(self):
        self.refresh()
        previous_map = self.card._map_html.content
        self.vm.update_doc(self.make_doc(connected=False))

        self.refresh()

        self.build_svg.assert_called_with(self.vm.topo_doc, None, '—')
        self.assertNotEqual(self.card._map_html.content, previous_map)

    def assert_robot_rendered_once(self, pose):
        self.build_robot_svg.assert_called_once()
        nodes, actual_pose = self.build_robot_svg.call_args.args
        self.assertEqual(list(nodes), list(self.vm.topo_doc.nodes))
        self.assertEqual(actual_pose, pose)


if __name__ == '__main__':
    unittest.main()
