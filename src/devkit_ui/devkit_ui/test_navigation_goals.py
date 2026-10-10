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
        """Store a result, resolution state, and pending completion callbacks."""
        self._result = result
        self._resolved = resolved
        self._callbacks = []

    def result(self):
        return self._result

    def add_done_callback(self, callback) -> None:
        """Call immediately if resolved; otherwise queue the completion callback."""
        if self._resolved:
            callback(self)
        else:
            self._callbacks.append(callback)

    def resolve(self, result) -> None:
        """Store the result, mark completion, and invoke queued callbacks."""
        self._result = result
        self._resolved = True
        for callback in self._callbacks:
            callback(self)


class GoalHandle:
    def __init__(self, accepted=True, success=True, finishes=True) -> None:
        """Configure goal acceptance, cancellation tracking, and result completion."""
        self.accepted = accepted
        self.cancel_goal_async = Mock()
        self.result_future = Future(
            SimpleNamespace(result=SimpleNamespace(success=success)), resolved=finishes)

    def get_result_async(self):
        return self.result_future


class ActionClient:
    """Stand-in action client; goals are accepted at once unless accept_now is False."""

    def __init__(self, *, ready=True, handle=None, accept_now=True) -> None:
        """Configure server readiness and acceptance timing while recording sent goals."""
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
        """Record the goal and feedback callback, then return the acceptance future."""
        self.sent_goals.append(goal)
        self.feedback_callback = feedback_callback
        return self.accept_future


def make_domain(client):
    """Build a navigation domain around the supplied fake action client."""
    ros = Mock(spec=RosGateway)
    ros.create_action_client.return_value = client
    with patch.object(nav_domain_module, 'GotoNode', SimpleNamespace(Goal=Goal)):
        domain = NavigationDomainService(ros)
    domain.log = ros.get_logger.return_value
    return domain


class TestNavigationDomainService(unittest.TestCase):
    def setUp(self):
        """Capture navigation updates and install a fake GotoNode interface."""
        self.updates = []
        self.on_update = self.updates.append
        self.patch = patch.object(nav_domain_module, 'GotoNode', SimpleNamespace(Goal=Goal))
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def statuses(self):
        """Return captured progress as status, navigating, and outcome tuples."""
        return [(u.status, u.navigating, u.outcome) for u in self.updates]

    def test_sends_goal_to_target_after_waiting_for_server(self):
        """Verify the target, server timeout, and initial navigating update."""
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)

        self.assertTrue(domain.send_goal('ROW_2_OUT', self.on_update))

        self.assertEqual(client.wait_timeout, 5.0)
        self.assertEqual(client.sent_goals[0].target, 'ROW_2_OUT')
        self.assertEqual(self.statuses(), [('→ ROW_2_OUT', True, None)])

    def test_server_timeout_reports_unavailable_and_allows_another_goal(self):
        """Verify server unavailability releases the domain for a later goal."""
        client = ActionClient(ready=False)
        domain = make_domain(client)

        domain.send_goal('ROW_4_IN', self.on_update)

        self.assertEqual(
            self.statuses(), [('action server not ready (5s timeout)', False, 'unavailable')])
        client.ready = True
        self.assertTrue(domain.send_goal('ROW_4_IN', self.on_update))
        self.assertEqual(len(client.sent_goals), 1)

    def test_missing_action_interface_reports_unavailable(self):
        """Verify a missing action interface reports unavailability without creating a client."""
        ros = Mock(spec=RosGateway)
        with patch.object(nav_domain_module, 'GotoNode', None):
            domain = NavigationDomainService(ros)

        domain.send_goal('ROW_5_IN', self.on_update)

        self.assertFalse(domain.available)
        ros.create_action_client.assert_not_called()
        self.assertEqual(
            self.statuses(), [('action unavailable (import failed)', False, 'unavailable')])

    def test_second_goal_is_refused_while_one_is_running(self):
        """Verify an active goal prevents a second submission and emits a warning."""
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)
        domain.send_goal('A', self.on_update)

        self.assertFalse(domain.send_goal('B', self.on_update))

        self.assertEqual(len(client.sent_goals), 1)
        domain._ros.get_logger.return_value.warn.assert_called_once()

    def test_rejected_goal_clears_state(self):
        """Verify rejection is reported and permits a subsequent goal."""
        client = ActionClient(handle=GoalHandle(accepted=False))
        domain = make_domain(client)

        domain.send_goal('ROW_6_IN', self.on_update)

        self.assertEqual(self.statuses()[-1], ('goal rejected', False, 'rejected'))
        self.assertTrue(domain.send_goal('ROW_6_IN', self.on_update))

    def test_result_reports_arrival_or_failure_and_frees_the_domain(self):
        """Verify success and failure both report terminal outcomes and release the domain."""
        for success, outcome in ((True, 'arrived'), (False, 'failed')):
            with self.subTest(success=success):
                self.updates.clear()
                domain = make_domain(ActionClient(handle=GoalHandle(success=success)))

                domain.send_goal('ROW_7_OUT', self.on_update)

                self.assertEqual(self.statuses()[-1], (outcome, False, outcome))
                self.assertTrue(domain.send_goal('ROW_7_OUT', self.on_update))

    def test_feedback_reports_current_node_then_falls_back_to_status(self):
        """Verify feedback prefers the current node and falls back to a status string."""
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)
        domain.send_goal('A', self.on_update)

        client.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(current_node='N1')))
        client.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(status='turning')))

        self.assertEqual(self.updates[-2].status, 'en route · N1')
        self.assertEqual(self.updates[-1].status, 'en route · turning')

    def test_cancel_accepted_goal_cancels_it_and_ignores_its_late_result(self):
        """Verify cancellation reports once and suppresses the cancelled goal's late result."""
        handle = GoalHandle(finishes=False)
        domain = make_domain(ActionClient(handle=handle))
        domain.send_goal('A', self.on_update)

        domain.cancel_goal()
        handle.result_future.resolve(SimpleNamespace(result=SimpleNamespace(success=False)))

        handle.cancel_goal_async.assert_called_once_with()
        self.assertEqual(self.statuses()[-1], ('cancelled', False, 'cancelled'))
        self.assertEqual(len(self.updates), 2)  # '→ A' and 'cancelled', nothing after

    def test_cancel_before_acceptance_cancels_when_the_goal_is_accepted(self):
        """Verify cancellation waits for acceptance and blocks other goals in the meantime."""
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

    def test_stale_callbacks_do_not_update_a_new_goal(self):
        """Verify feedback, acceptance, and result callbacks from an old goal are ignored."""
        # pylint: disable=protected-access
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)
        domain.send_goal('A', self.on_update)
        old_feedback = client.feedback_callback
        old_goal_id = domain._goal_id
        domain.cancel_goal()
        newer_updates = Mock()
        domain.send_goal('B', newer_updates)
        newer_updates.reset_mock()

        old_feedback(SimpleNamespace(feedback=SimpleNamespace(current_node='A')))
        domain._on_accepted(old_goal_id, Mock())
        domain._on_result(old_goal_id, Mock())

        newer_updates.assert_not_called()

    def test_callbacks_without_update_handler_are_ignored(self):
        """Verify callbacks tolerate a missing progress handler."""
        # pylint: disable=protected-access
        domain = make_domain(ActionClient())

        domain._on_feedback(domain._goal_id, Mock())
        domain._on_accepted(domain._goal_id, Mock())
        domain._on_result(domain._goal_id, Mock())

    def test_feedback_keeps_the_callback_captured_before_a_new_goal(self):
        """Verify racing goal replacement cannot redirect old feedback to the new handler."""
        client = ActionClient(handle=GoalHandle(finishes=False))
        domain = make_domain(client)
        domain.send_goal('A', self.on_update)
        newer_updates = Mock()

        class Feedback:
            @property
            def feedback(self):
                """Replace the active goal while feedback is read to exercise callback isolation."""
                domain.cancel_goal()
                domain.send_goal('B', newer_updates)
                newer_updates.reset_mock()
                return SimpleNamespace(current_node='A')

        client.feedback_callback(Feedback())

        newer_updates.assert_not_called()
        self.assertEqual(self.updates[-1].status, 'en route · A')

    def test_cancel_with_no_goal_does_nothing(self):
        """Verify cancellation without an active goal emits no updates."""
        domain = make_domain(ActionClient())

        domain.cancel_goal()

        self.assertEqual(self.updates, [])


class TestNavigationApplicationService(unittest.TestCase):
    def setUp(self):
        """Shorten polling intervals and provide a fake navigation action interface."""
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
        """Return application and domain services backed by the supplied fake client."""
        domain = make_domain(client)
        return NavigationApplicationService(domain), domain

    def test_wait_returns_true_only_on_arrival(self):
        """Verify waiting distinguishes arrival from a failed navigation result."""
        for success in (True, False):
            with self.subTest(success=success):
                app, _ = self.service(ActionClient(handle=GoalHandle(success=success)))
                self.assertEqual(app.navigate_and_wait('A', 5.0, lambda: False), success)

    def test_wait_returns_false_when_rejected_unavailable_or_busy(self):
        """Verify rejected, unavailable, and overlapping goals return false."""
        app, _ = self.service(ActionClient(handle=GoalHandle(accepted=False)))
        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: False))

        app, _ = self.service(ActionClient(ready=False))
        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: False))

        app, domain = self.service(ActionClient(handle=GoalHandle(finishes=False)))
        domain.send_goal('other', Mock())
        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: False))

    def test_wait_forwards_updates(self):
        """Verify blocking navigation forwards initial and terminal progress updates."""
        app, _ = self.service(ActionClient())
        seen = []

        app.navigate_and_wait('A', 5.0, lambda: False, seen.append)

        self.assertEqual([u.status for u in seen], ['→ A', 'arrived'])

    def test_abort_cancels_the_goal_and_returns_false(self):
        """Verify an abort request cancels the active goal and reports no arrival."""
        handle = GoalHandle(finishes=False)
        app, _ = self.service(ActionClient(handle=handle))

        self.assertFalse(app.navigate_and_wait('A', 5.0, lambda: True))

        handle.cancel_goal_async.assert_called_once_with()

    def test_timeout_cancels_the_goal_and_returns_false(self):
        """Verify expiry cancels the goal, logs a warning, and reports no arrival."""
        handle = GoalHandle(finishes=False)
        app, domain = self.service(ActionClient(handle=handle))

        self.assertFalse(app.navigate_and_wait('A', 0.03, lambda: False))

        handle.cancel_goal_async.assert_called_once_with()
        domain._ros.get_logger.return_value.warn.assert_called_once()

    def test_navigate_runs_off_the_calling_thread(self):
        """Verify navigate dispatches its worker through the substituted thread factory."""
        app, _ = self.service(ActionClient(handle=GoalHandle(success=True)))
        updates = []
        with patch.object(nav_app_module.threading, 'Thread',
                          lambda target, args, daemon: SimpleNamespace(
                              start=lambda: target(*args))):
            app.navigate('A', updates.append)

        self.assertEqual(updates[-1].outcome, 'arrived')


class TestRunViewModelNavigation(unittest.TestCase):
    def make_vm(self, available=True):
        """Return a run view model and mock navigation service with chosen availability."""
        nav = Mock()
        nav.available = available
        return RunViewModel(Mock(), navigation_app_service=nav), nav

    def test_navigate_to_marks_navigation_in_progress_and_starts_the_goal(self):
        """Verify navigation sets connecting state and passes the view-model callback."""
        vm, nav = self.make_vm()

        vm.navigate_to('ROW_2_OUT')

        self.assertEqual(vm.topo.nav_status, 'connecting → ROW_2_OUT…')
        self.assertTrue(vm.topo.navigating)
        nav.navigate.assert_called_once_with('ROW_2_OUT', vm._on_nav_update)

    def test_navigate_to_is_ignored_while_navigating(self):
        """Verify repeated navigation requests leave an active goal and its status unchanged."""
        vm, nav = self.make_vm()
        vm.topo.navigating = True
        vm.topo.nav_status = 'existing status'

        vm.navigate_to('ROW_3_IN')

        nav.navigate.assert_not_called()
        self.assertEqual(vm.topo.nav_status, 'existing status')

    def test_navigate_to_without_action_support_does_not_start_navigation(self):
        """Verify unavailable action support displays an error without starting a goal."""
        vm, nav = self.make_vm(available=False)

        vm.navigate_to('ROW_5_IN')

        nav.navigate.assert_not_called()
        self.assertEqual(vm.topo.nav_status, 'action unavailable (import failed)')
        self.assertFalse(vm.topo.navigating)

    def test_missing_navigation_service_is_unavailable(self):
        """Verify absent navigation services produce unavailable state and safe return values."""
        vm = RunViewModel(Mock())

        self.assertFalse(vm.navigation_available)
        vm.navigate_to('A')
        vm.cancel_navigation()

        self.assertFalse(vm.topo.navigating)
        self.assertEqual(vm.topo.nav_status, 'action unavailable (import failed)')
        self.assertFalse(vm.navigate_and_wait('A', 1.0, lambda: False))

    def test_updates_drive_status_and_flag(self):
        """Verify a progress callback updates both navigation status and the running flag."""
        vm, _ = self.make_vm()
        update = SimpleNamespace(status='arrived', navigating=False)

        vm._on_nav_update(update)

        self.assertEqual(vm.topo.nav_status, 'arrived')
        self.assertFalse(vm.topo.navigating)

    def test_cancel_and_wait_delegate_to_the_service(self):
        """Verify cancellation and blocking navigation delegate with the expected callback."""
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
