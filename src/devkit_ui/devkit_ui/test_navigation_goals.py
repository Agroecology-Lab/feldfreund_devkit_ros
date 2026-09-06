# NavigationHarness is built dynamically (see load_navigation_harness below) and its
# instance attributes are set in make_node() rather than __init__, so pylint's static
# analysis can't see them and raises spurious no-member/protected-access warnings.
# pylint: disable=no-member,protected-access,attribute-defined-outside-init,exec-used
import ast
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel


class ImmediateThread:
    def __init__(self, target, daemon=False) -> None:
        self.target = target
        self.daemon = daemon

    def start(self) -> None:
        self.target()


class Goal:
    def __init__(self) -> None:
        self.target = None


class Future:
    def __init__(self, result=None, invoke_callback=False) -> None:
        self._result = result
        self._invoke_callback = invoke_callback
        self.callback = None

    def result(self):
        return self._result

    def add_done_callback(self, callback) -> None:
        self.callback = callback
        if self._invoke_callback:
            callback(self)


class GoalHandle:
    def __init__(self, accepted, success=None) -> None:
        self.accepted = accepted
        result = SimpleNamespace(result=SimpleNamespace(success=success))
        self.result_future = Future(result, invoke_callback=True)

    def get_result_async(self):
        return self.result_future


class ActionClient:
    def __init__(self, *, ready=True, accepted=True, success=True, complete=True) -> None:
        self.ready = ready
        self.goal_handle = GoalHandle(accepted, success)
        self.future = Future(self.goal_handle, invoke_callback=complete)
        self.sent_goal = None
        self.feedback_callback = None

    def wait_for_server(self, timeout_sec):
        self.wait_timeout = timeout_sec
        return self.ready

    def send_goal_async(self, goal, feedback_callback):
        self.sent_goal = goal
        self.feedback_callback = feedback_callback
        return self.future


def load_navigation_harness():
    """Load only navigation methods, avoiding ui_node's ROS and web-server startup."""
    source_path = Path(__file__).with_name('ui_node.py')
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'NiceGuiNode'
    )
    method_names = {
        'send_nav_goal',
        '_nav_accepted',
        '_nav_feedback',
        '_nav_result',
        'cancel_nav_goal',
        '_send_goal_sync',
    }
    methods = [
        node for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name in method_names
    ]
    namespace = {
        '_ACTION_OK': True,
        'GotoNode': SimpleNamespace(Goal=Goal),
        'threading': SimpleNamespace(Thread=ImmediateThread, Event=threading.Event),
    }
    exec(compile(ast.fix_missing_locations(ast.Module(methods, type_ignores=[])), source_path, 'exec'), namespace)
    return type('NavigationHarness', (), {name: namespace[name] for name in method_names})


NavigationHarness = load_navigation_harness()


def make_node(action_client=None):
    node = NavigationHarness()
    node._run_vm = RunViewModel()
    node._global_vm = GlobalViewModel()
    node._nav_ac = action_client or ActionClient(complete=False)
    node._nav_goal_handle = None
    node._mission_cancel = False
    node.get_logger = Mock(return_value=Mock())
    return node


class TestSendNavGoal(unittest.TestCase):
    def setUp(self) -> None:
        NavigationHarness.send_nav_goal.__globals__['_ACTION_OK'] = True

    def test_submits_goal_and_marks_navigation_in_progress(self) -> None:
        action_client = ActionClient(complete=False)
        node = make_node(action_client)

        node.send_nav_goal('ROW_2_OUT')

        self.assertEqual(action_client.wait_timeout, 5.0)
        self.assertEqual(action_client.sent_goal.target, 'ROW_2_OUT')
        self.assertEqual(node._run_vm.topo.nav_status, '→ ROW_2_OUT')
        self.assertTrue(node._run_vm.topo.navigating)

    def test_rejects_reentrant_and_estopped_submissions_without_mutating_state(self) -> None:
        for navigating, estopped in ((True, False), (False, True), (True, True)):
            with self.subTest(navigating=navigating, estopped=estopped):
                action_client = ActionClient(complete=False)
                node = make_node(action_client)
                node._run_vm.topo.navigating = navigating
                node._run_vm.topo.nav_status = 'existing status'
                node._global_vm.soft_estop_active = estopped

                node.send_nav_goal('ROW_3_IN')

                self.assertIsNone(action_client.sent_goal)
                self.assertEqual(node._run_vm.topo.nav_status, 'existing status')
                self.assertEqual(node._run_vm.topo.navigating, navigating)
                node.get_logger().warn.assert_called_once()

    def test_action_server_timeout_clears_navigation_flag(self) -> None:
        node = make_node(ActionClient(ready=False))

        node.send_nav_goal('ROW_4_IN')

        self.assertEqual(node._run_vm.topo.nav_status, 'action server not ready (5s timeout)')
        self.assertFalse(node._run_vm.topo.navigating)

    def test_missing_action_support_does_not_start_navigation(self) -> None:
        node = make_node()
        NavigationHarness.send_nav_goal.__globals__['_ACTION_OK'] = False

        node.send_nav_goal('ROW_5_IN')

        self.assertEqual(node._run_vm.topo.nav_status, 'action unavailable (import failed)')
        self.assertFalse(node._run_vm.topo.navigating)


class TestSynchronousNavigationGoal(unittest.TestCase):
    def setUp(self) -> None:
        NavigationHarness._send_goal_sync.__globals__['_ACTION_OK'] = True

    def test_rejected_goal_clears_view_model_navigation_flag(self) -> None:
        node = make_node(ActionClient(accepted=False))

        result = node._send_goal_sync('ROW_6_IN')

        self.assertFalse(result)
        self.assertEqual(node._run_vm.topo.nav_status, 'goal rejected')
        self.assertFalse(node._run_vm.topo.navigating)
        self.assertFalse(hasattr(node, 'topo_navigating'))

    def test_result_updates_view_model_for_success_and_failure(self) -> None:
        for success, expected_status in ((True, 'arrived'), (False, 'failed')):
            with self.subTest(success=success):
                node = make_node(ActionClient(success=success))

                result = node._send_goal_sync('ROW_7_OUT')

                self.assertEqual(result, success)
                self.assertEqual(node._run_vm.topo.nav_status, expected_status)
                self.assertFalse(node._run_vm.topo.navigating)
                self.assertIsNone(node._nav_goal_handle)
                self.assertFalse(hasattr(node, 'topo_navigating'))

    def test_unavailable_server_never_marks_navigation_active(self) -> None:
        node = make_node(ActionClient(ready=False))

        result = node._send_goal_sync('ROW_8_OUT')

        self.assertFalse(result)
        self.assertFalse(node._run_vm.topo.navigating)
        node.get_logger().warn.assert_called_once_with('_send_goal_sync: action server not ready')


if __name__ == '__main__':
    unittest.main()
