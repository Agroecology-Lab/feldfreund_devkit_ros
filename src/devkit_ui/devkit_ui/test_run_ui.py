# FakeElement/FakeUi deliberately mirror nicegui's real API signatures (text=''
# before *args) and build fake elements dynamically, so pylint's static checks
# for keyword-before-vararg and attribute-defined-outside-init are expected
# false positives here, not real issues.
# pylint: disable=keyword-arg-before-vararg,attribute-defined-outside-init
import sys
import unittest
from dataclasses import dataclass, field
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch


class FakeElement:
    ui = None

    def __init__(self, text='', *args, **kwargs) -> None:
        self.text = text
        self.args = args
        self.kwargs = kwargs
        self.value = kwargs.get('value')
        self.enabled = True
        self.children = []
        self.handlers = {}
        self.binding = None
        self.class_names = ''
        self.properties = ''
        self.styles = []
        self.is_deleted = False
        self.is_open = False

    def __enter__(self):
        self.ui.context.append(self)
        return self

    def __exit__(self, *_args) -> None:
        self.ui.context.pop()

    def classes(self, value):
        self.class_names = value
        return self

    def props(self, value):
        self.properties = value
        return self

    def style(self, value):
        self.styles.append(value)
        return self

    def bind_value(self, source, attribute, forward=None):
        self.binding = SimpleNamespace(
            kind='value', source=source, attribute=attribute,
            transform=forward,
        )
        self.value = getattr(source, attribute)
        return self

    def bind_value_from(self, source, attribute, backward=None):
        self.binding = SimpleNamespace(
            kind='value', source=source, attribute=attribute,
            transform=backward,
        )
        self.refresh_binding()
        return self

    def bind_text_from(self, source, attribute, backward=None):
        self.binding = SimpleNamespace(
            kind='text', source=source, attribute=attribute,
            transform=backward,
        )
        self.refresh_binding()
        return self

    def bind_enabled_from(self, source, attribute, backward=None):
        self.binding = SimpleNamespace(
            kind='enabled', source=source, attribute=attribute,
            transform=backward,
        )
        self.refresh_binding()
        return self

    def refresh_binding(self):
        value = getattr(self.binding.source, self.binding.attribute)
        if self.binding.transform:
            value = self.binding.transform(value)
        setattr(self, self.binding.kind, value)
        return value

    def on(self, event, handler):
        self.handlers[event] = handler
        return self

    def set_value(self, value) -> None:
        self.value = value

    def set_text(self, value) -> None:
        self.text = value

    def clear(self) -> None:
        self.children.clear()

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False


class FakeDialog(FakeElement):
    def __init__(self, result) -> None:
        super().__init__()
        self.result = result

    def __await__(self):
        async def wait_for_result():
            return self.result

        return wait_for_result().__await__()


class FakeTimer:
    def __init__(self, interval, callback, once=False) -> None:
        self.interval = interval
        self.callback = callback
        self.once = once


class FakeUi:
    def __init__(self) -> None:
        self.context = []
        self.elements = []
        self.timers = []
        self.dialog_results = []

    def reset(self) -> None:
        self.context.clear()
        self.elements.clear()
        self.timers.clear()
        self.dialog_results.clear()

    def _element(self, kind, text='', *args, **kwargs):
        element = FakeElement(text, *args, **kwargs)
        element.kind = kind
        self.elements.append(element)
        if self.context:
            self.context[-1].children.append(element)
        return element

    def row(self, *args, **kwargs):
        return self._element('row', *args, **kwargs)

    def label(self, text='', *args, **kwargs):
        return self._element('label', text, *args, **kwargs)

    def input(self, *args, **kwargs):
        return self._element('input', *args, **kwargs)

    def number(self, *args, **kwargs):
        return self._element('number', *args, **kwargs)

    def toggle(self, *args, **kwargs):
        return self._element('toggle', *args, **kwargs)

    def button(self, text='', *args, **kwargs):
        element = self._element('button', text, *args, **kwargs)
        element.on_click = kwargs.get('on_click')
        return element

    def checkbox(self, text='', *args, **kwargs):
        return self._element('checkbox', text, *args, **kwargs)

    def html(self, text='', *args, **kwargs):
        return self._element('html', text, *args, **kwargs)

    def separator(self, *args, **kwargs):
        return self._element('separator', *args, **kwargs)

    def scroll_area(self, *args, **kwargs):
        return self._element('scroll_area', *args, **kwargs)

    def column(self, *args, **kwargs):
        return self._element('column', *args, **kwargs)

    def element(self, tag, *args, **kwargs):
        return self._element(tag, *args, **kwargs)

    def dialog(self):
        result = self.dialog_results.pop(0) if self.dialog_results else None
        dialog = FakeDialog(result)
        dialog.kind = 'dialog'
        self.elements.append(dialog)
        if self.context:
            self.context[-1].children.append(dialog)
        return dialog

    def timer(self, interval, callback, once=False):
        timer = FakeTimer(interval, callback, once)
        self.timers.append(timer)
        return timer

    def find(self, kind, text=None):
        return next(
            element for element in self.elements
            if element.kind == kind and (text is None or element.text == text)
        )


fake_ui = FakeUi()
FakeElement.ui = fake_ui
nicegui = ModuleType('nicegui')
nicegui.ui = fake_ui
fake_ui.card = FakeElement

with patch.dict(sys.modules, {'nicegui': nicegui}):
    from devkit_ui.constants import NAV_ACTION, ROW_ACTION
    from devkit_ui.pages.run.drop_node_card import DropNodeCard
    from devkit_ui.pages.run.navigation_sidebar import NavigationSidebar
    from devkit_ui.pages.run.row_discovery_card import RowDiscoveryCard
    from devkit_ui.view_models.global_view_model import GlobalViewModel
    from devkit_ui.view_models.run_view_model import RunViewModel


geometry_msgs = ModuleType('geometry_msgs')
geometry_msgs_msg = ModuleType('geometry_msgs.msg')
geometry_msgs_msg.Point32 = type('Point32', (), {})
geometry_msgs_msg.PolygonStamped = type('PolygonStamped', (), {})
rclpy = ModuleType('rclpy')
rclpy_qos = ModuleType('rclpy.qos')
rclpy_qos.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL='transient_local')
rclpy_qos.HistoryPolicy = SimpleNamespace(KEEP_LAST='keep_last')
rclpy_qos.ReliabilityPolicy = SimpleNamespace(RELIABLE='reliable')
rclpy_qos.QoSProfile = lambda **_kwargs: object()
yaml = ModuleType('yaml')
yaml.safe_load = Mock()
yaml.dump = Mock()

with patch.dict(sys.modules, {
    'geometry_msgs': geometry_msgs,
    'geometry_msgs.msg': geometry_msgs_msg,
    'nicegui': nicegui,
    'rclpy': rclpy,
    'rclpy.qos': rclpy_qos,
    'yaml': yaml,
}):
    from devkit_ui.obstacles import attach_nav_card


@dataclass
class Node:
    name: str
    meta: dict = field(default_factory=dict)


class TestRunViewModel(unittest.TestCase):
    def test_defaults_match_idle_run_state(self) -> None:
        state = RunViewModel()

        self.assertEqual(state.joystick.pose_lbl, 'no odom')
        self.assertEqual((state.node_map.map_svg, state.node_map.robot_svg), ('', ''))
        self.assertEqual(
            (state.track.interval, state.track.row_role, state.track.running),
            (5.0, 'entry', False),
        )
        self.assertEqual((state.topo.current_node, state.topo.nav_status), ('—', 'idle'))
        self.assertEqual((state.discovery.active, state.discovery.status), (False, 'idle'))

    def test_instances_do_not_share_nested_state(self) -> None:
        first = RunViewModel()
        second = RunViewModel()

        first.topo.selected_node = 'ROW_1'
        first.discovery.active = True

        self.assertIsNone(second.topo.selected_node)
        self.assertFalse(second.discovery.active)


class TestDropNodeCard(unittest.TestCase):
    def setUp(self) -> None:
        fake_ui.reset()
        self.state = RunViewModel.DropNode()
        self.topo = RunViewModel.Topo()
        self.on_drop = Mock()
        self.card = DropNodeCard(self.state, self.topo, self.on_drop)

    def test_row_id_binding_normalizes_numbers_and_blank_values(self) -> None:
        row_id_input = fake_ui.find('number')

        self.assertEqual(row_id_input.binding.transform('4'), 4)
        self.assertIsNone(row_id_input.binding.transform(''))
        self.assertIsNone(row_id_input.binding.transform(None))

    def test_hint_current_node_and_status_bindings_cover_boundary_states(self) -> None:
        self.assertEqual(self.card.row_hint.text, NAV_ACTION)
        self.assertIn('#8c959f', self.card.row_hint.styles[-1])
        self.assertEqual(self.card.current_node_lbl.text, 'no current node')

        self.state.row_id = 1
        self.assertEqual(self.card.row_hint.refresh_binding(), ROW_ACTION)
        self.assertIn('#0969da', self.card.row_hint.styles[-1])

        self.topo.current_node = 'ROW_1_IN'
        self.assertEqual(self.card.current_node_lbl.refresh_binding(), '→ ROW_1_IN')
        self.assertIn('#1a7f37', self.card.current_node_lbl.styles[-1])

        self.state.status = 'ERROR: no odometry'
        self.card.status_lbl.refresh_binding()
        self.assertIn('#cf222e', self.card.status_lbl.styles[-1])

    def test_drop_uses_latest_bound_state(self) -> None:
        self.state.name = 'ROW_2_OUT'
        self.state.row_id = 2
        self.state.row_role = 'exit'

        fake_ui.find('button', 'Drop').on_click()

        self.on_drop.assert_called_once_with('ROW_2_OUT', 2, 'exit')


class TestNavigationSidebar(unittest.TestCase):
    def setUp(self) -> None:
        fake_ui.reset()
        self.global_state = GlobalViewModel()
        self.topo = RunViewModel.Topo()
        self.on_go = Mock()
        self.on_cancel = Mock()
        self.on_delete = Mock()
        self.on_select = Mock()
        self.sidebar = NavigationSidebar(
            self.global_state,
            self.topo,
            self.on_go,
            self.on_cancel,
            self.on_delete,
            self.on_select,
        )

    def test_action_buttons_enforce_selection_navigation_and_estop_guards(self) -> None:
        go = fake_ui.find('button', 'Go')
        cancel = fake_ui.find('button', 'Cancel')
        delete = fake_ui.find('button', 'Delete Node')

        self.assertFalse(go.enabled)
        self.assertFalse(cancel.enabled)
        self.assertFalse(delete.enabled)

        self.topo.selected_node = 'ROW_1'
        self.assertTrue(go.refresh_binding())
        self.assertTrue(delete.refresh_binding())

        self.topo.navigating = True
        self.assertFalse(go.refresh_binding())
        self.assertTrue(cancel.refresh_binding())

        self.topo.navigating = False
        self.global_state.soft_estop_active = True
        self.assertFalse(go.refresh_binding())

    def test_status_bindings_distinguish_success_failure_error_and_idle(self) -> None:
        for status, color in (
            ('arrived', '#1a7f37'),
            ('failed', '#cf222e'),
            ('connecting', '#57606a'),
        ):
            self.topo.nav_status = status
            self.sidebar.nav_status.refresh_binding()
            self.assertIn(color, self.sidebar.nav_status.styles[-1])

        for status, color in (
            ('deleted ROW_1', '#1a7f37'),
            ('ERROR: not found', '#cf222e'),
            ('', '#57606a'),
        ):
            self.topo.delete_status = status
            self.sidebar.delete_status.refresh_binding()
            self.assertIn(color, self.sidebar.delete_status.styles[-1])

    def test_render_nodes_sorts_marks_and_selects_each_node(self) -> None:
        nodes = [
            Node('Z_STANDARD', {'gps_lat': 51.1, 'gps_lon': -2.2}),
            Node('A_ROW', {
                'row_id': 3,
                'row_role': 'entry',
                'gps_lat': 51.2,
                'gps_lon': -2.3,
            }),
        ]

        self.sidebar.render_nodes(nodes, selected_node='A_ROW')

        rendered = self.sidebar.node_col.children
        self.assertEqual(len(rendered), 2)
        self.assertIn('A_ROW  · row', rendered[0].text)
        self.assertIn('node-item sel row', rendered[0].text)
        self.assertIn('Row 3 entry | 51.2 -2.3', rendered[0].text)
        self.assertIn('Z_STANDARD', rendered[1].text)
        rendered[0].handlers['click'](None)
        rendered[1].handlers['click'](None)
        self.assertEqual(self.on_select.call_args_list[0].args, ('A_ROW',))
        self.assertEqual(self.on_select.call_args_list[1].args, ('Z_STANDARD',))

    def test_render_nodes_replaces_stale_entries(self) -> None:
        self.sidebar.render_nodes([Node('OLD')], selected_node=None)
        self.sidebar.render_nodes([Node('NEW')], selected_node=None)

        self.assertEqual(len(self.sidebar.node_col.children), 1)
        self.assertIn('NEW', self.sidebar.node_col.children[0].text)


class TestRowDiscoveryCard(unittest.IsolatedAsyncioTestCase):
    def setUp(self) -> None:
        fake_ui.reset()
        self.state = RunViewModel.Discovery()
        self.on_start = Mock()
        self.on_stop = Mock()
        RowDiscoveryCard(self.state, self.on_start, self.on_stop)
        self.checkbox = fake_ui.find('checkbox', 'Discovery mode')
        self.on_change = self.checkbox.handlers['update:model-value']

    async def test_start_requires_explicit_confirmation(self) -> None:
        fake_ui.dialog_results.append('cancel')

        await self.on_change(SimpleNamespace(args=True))

        self.assertFalse(self.checkbox.value)
        self.on_start.assert_not_called()

    async def test_confirm_starts_and_unticking_stops_discovery(self) -> None:
        fake_ui.dialog_results.append('go')

        await self.on_change(SimpleNamespace(args=True))
        await self.on_change(SimpleNamespace(args=False))

        self.on_start.assert_called_once_with()
        self.on_stop.assert_called_once_with()


class TestObstacleUndo(unittest.TestCase):
    def setUp(self) -> None:
        fake_ui.reset()
        self.node = SimpleNamespace(
            latest_gps=SimpleNamespace(latitude=51.45, longitude=-2.58),
            obstacle_status='',
        )
        self.manager = Mock()

    def test_marked_obstacle_can_be_undone(self) -> None:
        self.manager.add.return_value = 'OBS_1'
        attach_nav_card(self.node, self.manager)
        fake_ui.find('input').value = 'gate post'

        fake_ui.find('button', 'Mark here').on_click()

        self.manager.add.assert_called_once_with(
            'circle',
            lat=51.45,
            lon=-2.58,
            radius_m=0.5,
            name='gate post',
        )
        self.assertEqual(fake_ui.find('input').value, '')
        dialog = fake_ui.find('dialog')
        self.assertTrue(dialog.is_open)
        self.assertIn('position=bottom seamless', dialog.properties)

        fake_ui.find('button', 'Undo').on_click()

        self.manager.delete.assert_called_once_with('OBS_1')
        self.assertFalse(dialog.is_open)

    def test_undo_prompt_times_out_once_but_ignores_deleted_dialog(self) -> None:
        self.manager.add.return_value = 'OBS_2'
        attach_nav_card(self.node, self.manager)
        fake_ui.find('button', 'Mark here').on_click()
        dialog = fake_ui.find('dialog')
        timeout = next(timer for timer in fake_ui.timers if timer.once)

        self.assertEqual(timeout.interval, 5.0)
        timeout.callback()
        self.assertFalse(dialog.is_open)

        dialog.is_open = True
        dialog.is_deleted = True
        timeout.callback()
        self.assertTrue(dialog.is_open)

    def test_missing_gps_reports_error_without_creating_undo_prompt(self) -> None:
        self.node.latest_gps = None
        attach_nav_card(self.node, self.manager)

        fake_ui.find('button', 'Mark here').on_click()

        self.assertEqual(self.node.obstacle_status, 'ERROR: no GPS message yet')
        self.manager.add.assert_not_called()
        self.assertFalse(any(element.kind == 'dialog' for element in fake_ui.elements))


if __name__ == '__main__':
    unittest.main()
