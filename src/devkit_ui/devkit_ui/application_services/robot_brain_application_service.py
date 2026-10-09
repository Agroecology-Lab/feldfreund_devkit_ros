from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from devkit_ui.domain_services.robot_brain_domain_service import RobotBrainDomainService


class RobotBrainApplicationService:
    """User-intent commands for the robot brain (the ESP32 running Lizard)."""

    def __init__(self, robot_brain_service: RobotBrainDomainService) -> None:
        self._brain = robot_brain_service

    def enable(self) -> None:
        self._brain.send('enable')

    def disable(self) -> None:
        self._brain.send('disable')

    def reset(self) -> None:
        self._brain.send('reset')

    def restart(self) -> None:
        self._brain.send('restart')

    def configure(self) -> None:
        self._brain.send('configure')
