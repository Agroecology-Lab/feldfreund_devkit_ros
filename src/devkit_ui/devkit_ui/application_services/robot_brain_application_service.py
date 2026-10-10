from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from devkit_ui.domain_services.robot_brain_domain_service import RobotBrainDomainService


class RobotBrainApplicationService:
    """User-intent commands for the robot brain (the ESP32 running Lizard)."""

    def __init__(self, robot_brain_service: RobotBrainDomainService) -> None:
        """Store the domain service used to publish robot brain commands."""
        self._brain = robot_brain_service

    def enable(self) -> None:
        """Send the enable command to the robot brain."""
        self._brain.send('enable')

    def disable(self) -> None:
        """Send the disable command to the robot brain."""
        self._brain.send('disable')

    def reset(self) -> None:
        """Send the reset command to the robot brain."""
        self._brain.send('reset')

    def restart(self) -> None:
        """Send the restart command to the robot brain."""
        self._brain.send('restart')

    def configure(self) -> None:
        """Send the configure command to the robot brain."""
        self._brain.send('configure')
