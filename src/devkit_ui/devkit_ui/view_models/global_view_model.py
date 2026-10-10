from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from devkit_ui.application_services.drive_application_service import (
        DriveApplicationService,
    )


class GlobalViewModel:
    def __init__(self, drive_app_service: DriveApplicationService) -> None:
        """Initialize the view model with the soft emergency stop inactive."""
        self.soft_estop_active: bool = False
        self._drive_app_service = drive_app_service

    def toggle_estop(self) -> None:
        """Update soft estop state from the service result after publishing succeeds."""
        self.soft_estop_active = self._drive_app_service.toggle_estop(
            self.soft_estop_active)
