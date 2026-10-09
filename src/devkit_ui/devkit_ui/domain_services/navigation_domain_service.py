from __future__ import annotations

import threading
from collections.abc import Callable

from devkit_ui.ros_gateway import RosGateway
from devkit_ui.topo_results import NavUpdate

try:
    from topological_navigation_msgs.action import GotoNode
except ImportError:  # topological navigation not installed (e.g. unit tests)
    GotoNode = None

NAV_ACTION_NAME = 'topological_navigation'


class NavigationDomainService:
    """Send GotoNode goals to the topological navigation action server.

    One goal runs at a time. Progress is reported to the on_update callback given
    to send_goal as NavUpdate values, which carry no UI or view-model types.
    Callbacks arrive on the ROS executor thread.
    """

    def __init__(self, ros: RosGateway) -> None:
        self._ros = ros
        self._client = (
            ros.create_action_client(GotoNode, NAV_ACTION_NAME) if GotoNode is not None else None)
        self._lock = threading.Lock()
        self._on_update: Callable[[NavUpdate], None] | None = None
        self._goal_handle = None
        self._goal_id = 0
        self._active = False
        self._cancel_requested = False

    @property
    def available(self) -> bool:
        """Whether the navigation action interface could be imported."""
        return self._client is not None

    def send_goal(self, target: str, on_update: Callable[[NavUpdate], None],
                  server_timeout: float = 5.0) -> bool:
        """Send a goal to the node named target.

        Blocks up to server_timeout seconds waiting for the action server, so call
        it from a worker thread. Returns False if no goal was sent because another
        goal is already running; every other outcome is reported through on_update.
        """
        if self._client is None:
            on_update(NavUpdate('action unavailable (import failed)', False, 'unavailable'))
            return True
        with self._lock:
            if self._active:
                self._ros.get_logger().warn(
                    f'send_goal: {target!r} rejected, a goal is already running')
                return False
            self._active = True
            self._cancel_requested = False
            self._goal_id += 1
            goal_id = self._goal_id
            self._on_update = on_update

        if not self._client.wait_for_server(timeout_sec=server_timeout):
            self._finish(goal_id)
            on_update(NavUpdate(
                f'action server not ready ({server_timeout:g}s timeout)', False, 'unavailable'))
            return True

        goal = GotoNode.Goal()
        goal.target = target
        on_update(NavUpdate(f'→ {target}', True))
        future = self._client.send_goal_async(
            goal, feedback_callback=lambda msg: self._on_feedback(goal_id, msg))
        future.add_done_callback(lambda fut: self._on_accepted(goal_id, fut))
        return True

    def cancel_goal(self, reason: str = '') -> None:
        """Cancel the running goal.

        If the goal was accepted it is cancelled now. If it is still being sent, it
        is cancelled the moment the server accepts it, and the navigating state is
        kept until then so a second goal cannot start in between.
        """
        with self._lock:
            handle = self._goal_handle
            on_update = self._on_update
            if handle is not None:
                self._goal_handle = None
                self._active = False
                self._goal_id += 1  # drop late callbacks from the goal being cancelled
            elif self._active:
                self._cancel_requested = True
            else:
                return
        if reason:
            self._ros.get_logger().warn(f'cancel_goal: {reason}')
        if handle is not None:
            handle.cancel_goal_async()
            on_update(NavUpdate('cancelled', False, 'cancelled'))
        else:
            on_update(NavUpdate('cancelling…', True))

    def _finish(self, goal_id: int) -> bool:
        """Clear the running goal's state; False if goal_id is no longer current."""
        with self._lock:
            if goal_id != self._goal_id:
                return False
            self._goal_handle = None
            self._active = False
            self._cancel_requested = False
            return True

    def _on_feedback(self, goal_id: int, feedback_msg) -> None:
        if goal_id != self._goal_id:
            return
        feedback = feedback_msg.feedback
        where = getattr(feedback, 'current_node', None) or getattr(feedback, 'status', '…')
        self._on_update(NavUpdate(f'en route · {where}', True))

    def _on_accepted(self, goal_id: int, future) -> None:
        handle = future.result()
        if not handle.accepted:
            if self._finish(goal_id):
                self._on_update(NavUpdate('goal rejected', False, 'rejected'))
            return
        with self._lock:
            if goal_id != self._goal_id:
                return
            self._goal_handle = handle
            cancel_now = self._cancel_requested
            self._cancel_requested = False
        if cancel_now:
            self._on_update(NavUpdate('cancelling…', True))
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda fut: self._on_result(goal_id, fut))

    def _on_result(self, goal_id: int, future) -> None:
        success = getattr(future.result().result, 'success', True)
        if self._finish(goal_id):
            outcome = 'arrived' if success else 'failed'
            self._on_update(NavUpdate(outcome, False, outcome))
