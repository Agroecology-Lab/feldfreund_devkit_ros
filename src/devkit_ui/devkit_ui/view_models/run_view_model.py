from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from devkit_ui.constants import ROW_ACTION

if TYPE_CHECKING:
    from devkit_ui.application_services.drive_application_service import (
        DriveApplicationService,
    )


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

    def __init__(self, drive_app_service: DriveApplicationService) -> None:
        """Initialize the run screen state with default values for each view-model component."""
        self.joystick = self.Joystick()
        self.node_map = self.NodeMap()
        self.track = self.Track()
        self.drop_node = self.DropNode()
        self.topo = self.Topo()
        self.discovery = self.Discovery()

        self._drive_app_service = drive_app_service

    def move_joystick(self, x: float, y: float) -> None:
        self._drive_app_service.move_joystick(x, y)

    def stop_joystick(self) -> None:
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
