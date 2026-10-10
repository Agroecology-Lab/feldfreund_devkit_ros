from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import NavSatFix

    from devkit_ui.application_services.telemetry_application_service import (
        TelemetryApplicationService,
    )


class TelemetryViewModel:
    """UI-facing sensor state for the Run and System screens."""

    def __init__(self, app_service: TelemetryApplicationService) -> None:
        """Initialize measured velocities at zero and bind to the telemetry service."""
        self._app_service = app_service
        self.linear_velocity: float = 0.0
        self.angular_velocity: float = 0.0

    def refresh(self) -> None:
        """Update the displayed velocities from the latest odometry.

        The last values are kept while no odometry is available.
        """
        velocity = self._app_service.measured_velocity()
        if velocity is not None:
            self.linear_velocity, self.angular_velocity = velocity

    @property
    def odom(self) -> Odometry | None:
        """Return the latest odometry, or None before any has arrived."""
        return self._app_service.latest_odom

    @property
    def gps(self) -> NavSatFix | None:
        """Return the latest GNSS fix, or None before any has arrived."""
        return self._app_service.latest_gps

    @property
    def battery_text(self) -> str:
        """Return the battery level and voltage for display, or a dash without data."""
        battery = self._app_service.latest_battery
        if battery is None:
            return '—'
        return f'{battery.percentage * 100:.1f}%  {battery.voltage:.1f} V'

    @property
    def bumper_front_top_active(self) -> bool:
        """Return whether the front-top bumper is pressed."""
        return self._app_service.bumper_front_top_active

    @property
    def bumper_front_bottom_active(self) -> bool:
        """Return whether the front-bottom bumper is pressed."""
        return self._app_service.bumper_front_bottom_active

    @property
    def bumper_back_active(self) -> bool:
        """Return whether the rear bumper is pressed."""
        return self._app_service.bumper_back_active

    @property
    def estop_front_active(self) -> bool:
        """Return whether the front hardware e-stop is active."""
        return self._app_service.estop_front_active

    @property
    def estop_back_active(self) -> bool:
        """Return whether the rear hardware e-stop is active."""
        return self._app_service.estop_back_active

    def robot_pose(self) -> tuple[float, float, float] | None:
        """Return (x, y, yaw) in the map frame, or None while the pose is unavailable."""
        return self._app_service.robot_pose()
