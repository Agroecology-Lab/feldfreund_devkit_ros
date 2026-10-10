from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState, NavSatFix

    from devkit_ui.domain_services.telemetry_domain_service import TelemetryDomainService


class TelemetryApplicationService:
    """Read-only access to the robot's latest sensor state.

    Hands out the values the telemetry domain service has accepted, plus the few derived
    readings the UI needs, without exposing any ROS wiring.
    """

    def __init__(self, telemetry_service: TelemetryDomainService) -> None:
        """Store the domain service that owns the telemetry state."""
        self._telemetry = telemetry_service

    @property
    def latest_odom(self) -> Odometry | None:
        """Return the odometry currently driving the UI, or None before any has arrived."""
        return self._telemetry.latest_odom

    @property
    def latest_gps(self) -> NavSatFix | None:
        """Return the cached GNSS message, or None before any has been cached.

        May contain an invalid real fix or the simulation fallback; freshness is not checked.
        """
        return self._telemetry.latest_gps

    @property
    def latest_battery(self) -> BatteryState | None:
        """Return the latest battery state, or None before any has arrived."""
        return self._telemetry.latest_battery

    @property
    def bumper_front_top_active(self) -> bool:
        """Return whether the front-top bumper is pressed."""
        return self._telemetry.bumper_front_top_active

    @property
    def bumper_front_bottom_active(self) -> bool:
        """Return whether the front-bottom bumper is pressed."""
        return self._telemetry.bumper_front_bottom_active

    @property
    def bumper_back_active(self) -> bool:
        """Return whether the rear bumper is pressed."""
        return self._telemetry.bumper_back_active

    @property
    def estop_front_active(self) -> bool:
        """Return whether the front hardware e-stop is active."""
        return self._telemetry.estop_front_active

    @property
    def estop_back_active(self) -> bool:
        """Return whether the rear hardware e-stop is active."""
        return self._telemetry.estop_back_active

    def measured_velocity(self) -> tuple[float, float] | None:
        """Return (linear x in m/s, angular z in rad/s) from the latest odometry.

        Values are in the odometry's child frame; return None without odometry.
        """
        odom = self._telemetry.latest_odom
        if odom is None:
            return None
        return odom.twist.twist.linear.x, odom.twist.twist.angular.z

    def robot_pose(self) -> tuple[float, float, float] | None:
        """Return (x, y, yaw) in the map frame, in meters, meters, and radians.

        Return None for an unavailable TF lookup or a transform over two node-clock seconds old.
        Other lookup errors propagate from the telemetry service.
        """
        return self._telemetry.robot_pose()
