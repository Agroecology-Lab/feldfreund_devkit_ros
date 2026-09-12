from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from devkit_ui.domain_services.drive_domain_service import DriveDomainService


class DriveApplicationService:
    """Exposes drive-related user-intent commands.

    Serves as the primary entry point into the drive domain service layer,
    handling lightweight translation or formatting while keeping business logic
    encapsulated within DriveDomainService.
    """

    def __init__(self, drive_service: DriveDomainService) -> None:
        self._drive = drive_service

    def move_joystick(self, x: float, y: float) -> None:
        self._drive.send_speed(x, y)

    def toggle_estop(self, current: bool) -> bool:
        return self._drive.toggle_estop(current)
