from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from devkit_ui.constants import ROW_ACTION

if TYPE_CHECKING:
    from devkit_ui.application_services.drive_application_service import (
        DriveApplicationService,
    )
    from devkit_ui.application_services.navigation_application_service import (
        NavigationApplicationService,
    )
    from devkit_ui.application_services.row_discovery_application_service import (
        RowDiscoveryApplicationService,
    )
    from devkit_ui.topo_results import DiscoveryResult, NavUpdate


class RunViewModel:
    @dataclass
    class Joystick:
        pose_lbl: str = 'no odom'

    @dataclass
    class NodeMap:
        robot_pose: tuple | None = None

    @dataclass
    class Track:
        prefix: str = ''
        interval: float = 5.0
        row_id: int | None = None
        row_role: str = 'entry'
        running: bool = False
        status: str = ''

    @dataclass
    class DropNode:
        name: str = ''
        row_id: int | None = None
        row_role: str = 'entry'
        row_hint: str = ''
        status: str = ''
        row_action: str = ROW_ACTION

    @dataclass
    class Topo:
        navigating: bool = False
        nav_status: str = 'idle'
        delete_status: str = ''

    @dataclass
    class Discovery:
        active: bool = False
        status: str = 'idle'

    def __init__(self, drive_app_service: DriveApplicationService,
                 discovery_app_service: RowDiscoveryApplicationService | None = None,
                 navigation_app_service: NavigationApplicationService | None = None) -> None:
        """Initialize the run screen state with default values for each view-model component."""
        self.joystick = self.Joystick()
        self.node_map = self.NodeMap()
        self.track = self.Track()
        self.drop_node = self.DropNode()
        self.topo = self.Topo()
        self.discovery = self.Discovery()

        self._drive_app_service = drive_app_service
        self._discovery_app_service = discovery_app_service
        self._navigation_app_service = navigation_app_service

        if discovery_app_service is not None:
            discovery_app_service.register_status_callback(self._on_discovery_status)

    def move_joystick(self, x: float, y: float) -> None:
        """Forward linear x and turn y commands to the drive application service."""
        self._drive_app_service.move_joystick(x, y)

    def stop_joystick(self) -> None:
        """Request zero linear and angular speed through the drive service."""
        self.move_joystick(0.0, 0.0)

    def update_pose_label(self, odom, gps) -> None:
        """Refresh the joystick pose label from latest odometry/GPS."""
        if odom is not None:
            px = odom.pose.pose.position.x
            py = odom.pose.pose.position.y
            gps_str = ''
            if gps is not None and gps.status.status >= 0:
                gps_str = f'\n{gps.latitude:.5f}\n{gps.longitude:.5f}'
            self.joystick.pose_lbl = f'({px:.2f}, {py:.2f}){gps_str}'
        else:
            self.joystick.pose_lbl = 'no odom'

    # ── row discovery ─────────────────────────────────────────────────────

    def start_discovery(self) -> None:
        """Ask the row discovery node to start; the outcome lands in self.discovery."""
        if self._discovery_app_service is None:
            self.discovery.status = 'ERROR: discovery service unavailable'
            return
        self.discovery.status = 'starting…'
        self._discovery_app_service.start(self._on_discovery_result)

    def stop_discovery(self) -> None:
        """Ask the row discovery node to stop; the outcome lands in self.discovery."""
        if self._discovery_app_service is not None:
            self._discovery_app_service.stop(self._on_discovery_result)

    def _on_discovery_status(self, status: str) -> None:
        """Replace the displayed discovery status with the latest message."""
        self.discovery.status = status

    def _on_discovery_result(self, result: DiscoveryResult) -> None:
        """Apply discovery status, preserving the active flag when state is unknown."""
        if result.active is not None:
            self.discovery.active = result.active
        self.discovery.status = result.status

    # ── navigation ────────────────────────────────────────────────────────

    def navigate_to(self, target: str) -> None:
        """Start navigating to a topology node. Ignored while a goal is already running."""
        if self.topo.navigating:
            return
        if self._navigation_app_service is None or not self._navigation_app_service.available:
            self.topo.nav_status = 'action unavailable (import failed)'
            return
        self.topo.nav_status = f'connecting → {target}…'
        self.topo.navigating = True
        self._navigation_app_service.navigate(target, self._on_nav_update)

    def cancel_navigation(self) -> None:
        """Cancel the running navigation goal, if any."""
        if self._navigation_app_service is not None:
            self._navigation_app_service.cancel()

    def navigate_and_wait(self, target: str, timeout_sec: float,
                          should_abort: Callable[[], bool]) -> bool:
        """Navigate to a topology node and block until it ends. Use from a worker thread."""
        if self._navigation_app_service is None:
            return False
        return self._navigation_app_service.navigate_and_wait(
            target, timeout_sec, should_abort, self._on_nav_update)

    @property
    def navigation_available(self) -> bool:
        """Whether the navigation action interface is available."""
        return self._navigation_app_service is not None and self._navigation_app_service.available

    def _on_nav_update(self, update: NavUpdate) -> None:
        """Apply navigation progress to the displayed status and running flag."""
        self.topo.nav_status = update.status
        self.topo.navigating = update.navigating
