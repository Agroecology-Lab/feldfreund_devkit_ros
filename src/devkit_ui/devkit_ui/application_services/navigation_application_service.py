from __future__ import annotations

import threading
from collections.abc import Callable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from devkit_ui.domain_services.navigation_domain_service import NavigationDomainService
    from devkit_ui.topo_results import NavUpdate

_POLL_INTERVAL_S = 0.25
_CANCEL_SETTLE_S = 2.0


class NavigationApplicationService:
    """User-intent commands for navigating to topology nodes."""

    def __init__(self, nav_service: NavigationDomainService) -> None:
        self._nav = nav_service

    @property
    def available(self) -> bool:
        return self._nav.available

    def navigate(self, target: str, on_update: Callable[[NavUpdate], None]) -> None:
        """Start navigating to target without blocking; progress goes to on_update."""
        threading.Thread(
            target=self._nav.send_goal, args=(target, on_update, 5.0), daemon=True).start()

    def cancel(self) -> None:
        """Cancel the running navigation goal."""
        self._nav.cancel_goal()

    def navigate_and_wait(self, target: str, timeout_sec: float,
                          should_abort: Callable[[], bool],
                          on_update: Callable[[NavUpdate], None] | None = None) -> bool:
        """Navigate to target and block until it ends. Call from a worker thread.

        Returns True only if the robot arrived. Returns False if the goal failed, was
        rejected or cancelled, the action server was unavailable, another goal was
        already running, timeout_sec ran out, or should_abort() became true (which
        cancels the goal).
        """
        done = threading.Event()
        outcome: list[str | None] = [None]

        def _on_update(update: NavUpdate) -> None:
            if on_update is not None:
                on_update(update)
            if update.outcome is not None:
                outcome[0] = update.outcome
                done.set()

        if not self._nav.send_goal(target, _on_update, server_timeout=10.0):
            return False

        remaining = timeout_sec
        while not done.wait(timeout=_POLL_INTERVAL_S):
            remaining -= _POLL_INTERVAL_S
            if remaining <= 0:
                self._nav.cancel_goal(f'timeout waiting for {target}')
                return False
            if should_abort():
                self._nav.cancel_goal()
                done.wait(timeout=_CANCEL_SETTLE_S)
                return False
        return outcome[0] == 'arrived'
