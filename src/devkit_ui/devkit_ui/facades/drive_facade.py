from __future__ import annotations

from devkit_ui.services.drive_service import DriveService


class DriveFacade:
    """Exposes drive-related user-intent commands.

    Serves as the primary entry point into the drive service layer, handling
    lightweight translation or formatting while keeping business logic encapsulated
    within DriveService.
    """

    def __init__(self, drive_service: DriveService) -> None:
        self._drive = drive_service

    def move_joystick(self, x: float, y: float) -> None:
        self._drive.send_speed(x, y)

    def toggle_estop(self, current: bool) -> bool:
        return self._drive.toggle_estop(current)
