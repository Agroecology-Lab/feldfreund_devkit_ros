"""Navigation goals through the domain service, application service and run view model."""
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.application_services import navigation_application_service as nav_app_module
from devkit_ui.application_services.navigation_application_service import (
    NavigationApplicationService,
)
from devkit_ui.view_models.run_view_model import RunViewModel

_rclpy_node = ModuleType('rclpy.node')
_rclpy_node.Node = type('Node', (), {})

with patch.dict(sys.modules, {'rclpy': ModuleType('rclpy'), 'rclpy.node': _rclpy_node}):
    from devkit_ui.domain_services import navigation_domain_service as nav_domain_module
    from devkit_ui.domain_services.navigation_domain_service import NavigationDomainService
    from devkit_ui.ros_gateway import RosGateway


class Goal:
    def __init__(self) -> None:
        self.target = None


class Future:
    """Calls its callback at once if already resolved, otherwise when resolve() runs."""

    def __init__(self, result=None, resolved=False) -> None:
        self._result = result
        self._resolved = resolved
        self._callbacks = []

    def result(self):
        return self._result

    def add_done_callback(self, callback) -> None:
        if self._resolved:
            callback(self)
        else:
            self._callbacks.append(callback)

    def resolve(self, result) -> None:
        self._result = result
        self._resolved = True
        for callback in self._callbacks:
            callback(self)


class GoalHandle:
    def __init__(self, accepted=True, success=True, finishes=True) -> None:
        self.accepted = accepted
        self.cancel_goal_async = Mock()
        self.result_future = Future(
            SimpleNamespace(result=SimpleNamespace(success=success)), resolved=finishes)

    def get_result_async(self):
        return self.result_future


class ActionClient:
    """Stand-in action client; goals are accepted at once unless accept_now is False."""

    def __init__(self, *, ready=True, handle=None, accept_now=True) -> None:
        self.ready = ready
        self.handle = handle or GoalHandle()
        self.accept_future = Future(self.handle, resolved=accept_now)
        self.wait_timeout = None
        self.sent_goals = []
        self.feedback_callback = None

    def wait_for_server(self, timeout_sec):
        self.wait_timeout = timeout_sec
        return self.ready

    def send_goal_async(self, goal, feedback_callback):
        self.sent_goals.append(goal)
        self.feedback_callback = feedback_callback
        return self.accept_future


def make_domain(client):
    ros = Mock(spec=RosGateway)
    ros.create_action_client.return_value = client
    with patch.object(nav_domain_module, 'GotoNode', SimpleNamespace(Goal=Goal)):
        domain = NavigationDomainService(ros)
    domain.log = ros.get_logger.return_value
    return domain


class TestNavigationDomainService(unittest.TestCase):
    def setUp(self):
        self.updates = []
        self.on_update = self.updates.append
        self.patch = patch.object(nav_domain_module, 'GotoNode', SimpleNamespace(Goal=Goal))
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def statuses(self):
        return [(u.status, u.navigating, u.outcome) for u in self.updates]

    def test_sends_goal_to_target_after_waiting_for_server(self):
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)

        self.assertTrue(domain.send_goal('ROW_2_OUT', self.on_update))

        self.assertEqual(client.wait_timeout, 5.0)
        self.assertEqual(client.sent_goals[0].target, 'ROW_2_OUT')
        self.assertEqual(self.statuses(), [('→ ROW_2_OUT', True, None)])

    def test_server_timeout_reports_unavailable_and_allows_another_goal(self):
        client = ActionClient(ready=False)
        domain = make_domain(client)

        domain.send_goal('ROW_4_IN', self.on_update)

        self.assertEqual(
            self.statuses(), [('action server not ready (5s timeout)', False, 'unavailable')])
        client.ready = True
        self.assertTrue(domain.send_goal('ROW_4_IN', self.on_update))
        self.assertEqual(len(client.sent_goals), 1)

    def test_missing_action_interface_reports_unavailable(self):
        ros = Mock(spec=RosGateway)
        with patch.object(nav_domain_module, 'GotoNode', None):
            domain = NavigationDomainService(ros)

        domain.send_goal('ROW_5_IN', self.on_update)

        self.assertFalse(domain.available)
        ros.create_action_client.assert_not_called()
        self.assertEqual(
            self.statuses(), [('action unavailable (import failed)', False, 'unavailable')])

    def test_second_goal_is_refused_while_one_is_running(self):
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)
        domain.send_goal('A', self.on_update)

        self.assertFalse(domain.send_goal('B', self.on_update))

        self.assertEqual(len(client.sent_goals), 1)
        domain._ros.get_logger.return_value.warn.assert_called_once()

    def test_rejected_goal_clears_state(self):
        client = ActionClient(handle=GoalHandle(accepted=False))
        domain = make_domain(client)

        domain.send_goal('ROW_6_IN', self.on_update)

        self.assertEqual(self.statuses()[-1], ('goal rejected', False, 'rejected'))
        self.assertTrue(domain.send_goal('ROW_6_IN', self.on_update))

    def test_result_reports_arrival_or_failure_and_frees_the_domain(self):
        for success, outcome in ((True, 'arrived'), (False, 'failed')):
            with self.subTest(success=success):
                self.updates.clear()
                domain = make_domain(ActionClient(handle=GoalHandle(success=success)))

                domain.send_goal('ROW_7_OUT', self.on_update)

                self.assertEqual(self.statuses()[-1], (outcome, False, outcome))
                self.assertTrue(domain.send_goal('ROW_7_OUT', self.on_update))

    def test_feedback_reports_current_node_then_falls_back_to_status(self):
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)
        domain.send_goal('A', self.on_update)

        client.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(current_node='N1')))
        client.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(status='turning')))

        self.assertEqual(self.updates[-2].status, 'en route · N1')
        self.assertEqual(self.updates[-1].status, 'en route · turning')

    def test_cancel_accepted_goal_cancels_it_and_ignores_its_late_result(self):
        handle = GoalHandle(finishes=False)
        domain = make_domain(ActionClient(handle=handle))
        domain.send_goal('A', self.on_update)

        domain.cancel_goal()
        handle.result_future.resolve(SimpleNamespace(result=SimpleNamespace(success=False)))

        handle.cancel_goal_async.assert_called_once_with()
        self.assertEqual(self.statuses()[-1], ('cancelled', False, 'cancelled'))
        self.assertEqual(len(self.updates), 2)  # '→ A' and 'cancelled', nothing after

    def test_cancel_before_acceptance_cancels_when_the_goal_is_accepted(self):
        handle = GoalHandle(finishes=False)
        client = ActionClient(handle=handle, accept_now=False)
        domain = make_domain(client)
        domain.send_goal('A', self.on_update)

        domain.cancel_goal()

        self.assertEqual(self.statuses()[-1], ('cancelling…', True, None))
        self.assertFalse(domain.send_goal('B', self.on_update))
        handle.cancel_goal_async.assert_not_called()
        client.accept_future.resolve(handle)
        handle.cancel_goal_async.assert_called_once_with()

    def test_cancel_with_no_goal_does_nothing(self):
        domain = make_domain(ActionClient())

        domain.cancel_goal()

        self.assertEqual(self.updates, [])


class TestNavigationApplicationService(unittest.TestCase):
    def setUp(self):
        patcher = patch.object(nav_app_module, '_POLL_INTERVAL_S', 0.01)
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = patch.object(nav_app_module, '_CANCEL_SETTLE_S', 0.01)
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = patch.object(nav_domain_module, 'GotoNode', SimpleNamespace(Goal=Goal))
        patcher.start()
        self.addCleanup(patcher.stop)

    def service(self, client):
        domain = make_domain(client)
        return NavigationApplicationService(domain), domain

    def test_wait_returns_true_only_on_arrival(self):
        for success in (True, False):
            with self.subTest(success=success):
                app, _ = self.service(ActionClient(handle=GoalHandle(success=success)))
                self.assertEqual(app.navigate_and_wait('A', 5.0, lambda: False), success)

    def test_wait_returns_false_when_rejected_unavailable_or_busy(self):
        app, _ = self.service(ActionClient(handle=GoalHandle(accepted=False)))
        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: False))

        app, _ = self.service(ActionClient(ready=False))
        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: False))

        app, domain = self.service(ActionClient(handle=GoalHandle(finishes=False)))
        domain.send_goal('other', Mock())
        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: False))

    def test_wait_forwards_updates(self):
        app, _ = self.service(ActionClient())
        seen = []

        app.navigate_and_wait('A', 5.0, lambda: False, seen.append)

        self.assertEqual([u.status for u in seen], ['→ A', 'arrived'])

    def test_abort_cancels_the_goal_and_returns_false(self):
        handle = GoalHandle(finishes=False)
        app, _ = self.service(ActionClient(handle=handle))

        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: True))

        handle.cancel_goal_async.assert_called_once_with()

    def test_timeout_cancels_the_goal_and_returns_false(self):
        handle = GoalHandle(finishes=False)
        app, domain = self.service(ActionClient(handle=handle))

        self.assertFalse(app.navigate_and_wait('A', 0.03, lambda: False))

        handle.cancel_goal_async.assert_called_once_with()
        domain._ros.get_logger.return_value.warn.assert_called_once()

    def test_navigate_runs_off_the_calling_thread(self):
        app, _ = self.service(ActionClient(handle=GoalHandle(success=True)))
        updates = []
        with patch.object(nav_app_module.threading, 'Thread',
                          lambda target, args, daemon: SimpleNamespace(
                              start=lambda: target(*args))):
            app.navigate('A', updates.append)

        self.assertEqual(updates[-1].outcome, 'arrived')


class TestRunViewModelNavigation(unittest.TestCase):
    def make_vm(self, available=True):
        nav = Mock()
        nav.available = available
        return RunViewModel(Mock(), navigation_app_service=nav), nav

    def test_navigate_to_marks_navigation_in_progress_and_starts_the_goal(self):
        vm, nav = self.make_vm()

        vm.navigate_to('ROW_2_OUT')

        self.assertEqual(vm.topo.nav_status, 'connecting → ROW_2_OUT…')
        self.assertTrue(vm.topo.navigating)
        nav.navigate.assert_called_once_with('ROW_2_OUT', vm._on_nav_update)

    def test_navigate_to_is_ignored_while_navigating(self):
        vm, nav = self.make_vm()
        vm.topo.navigating = True
        vm.topo.nav_status = 'existing status'

        vm.navigate_to('ROW_3_IN')

        nav.navigate.assert_not_called()
        self.assertEqual(vm.topo.nav_status, 'existing status')

    def test_navigate_to_without_action_support_does_not_start_navigation(self):
        vm, nav = self.make_vm(available=False)

        vm.navigate_to('ROW_5_IN')

        nav.navigate.assert_not_called()
        self.assertEqual(vm.topo.nav_status, 'action unavailable (import failed)')
        self.assertFalse(vm.topo.navigating)

    def test_updates_drive_status_and_flag(self):
        vm, _ = self.make_vm()
        update = SimpleNamespace(status='arrived', navigating=False)

        vm._on_nav_update(update)

        self.assertEqual(vm.topo.nav_status, 'arrived')
        self.assertFalse(vm.topo.navigating)

    def test_cancel_and_wait_delegate_to_the_service(self):
        vm, nav = self.make_vm()
        nav.navigate_and_wait.return_value = True

        vm.cancel_navigation()
        result = vm.navigate_and_wait('A', 9.0, lambda: False)

        nav.cancel.assert_called_once_with()
        self.assertTrue(result)
        self.assertEqual(nav.navigate_and_wait.call_args.args[:2], ('A', 9.0))
        self.assertEqual(nav.navigate_and_wait.call_args.args[3], vm._on_nav_update)


if __name__ == '__main__':
    unittest.main()
