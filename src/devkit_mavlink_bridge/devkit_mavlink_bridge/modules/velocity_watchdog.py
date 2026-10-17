"""Command-age watchdog for the cmd_vel -> MAVLink path.

Plain Python, no rclpy or pymavlink, so it can be unit tested without ROS.

ArduPilot Rover in GUIDED mode keeps driving the last velocity target until
GUID_TIMEOUT (default 3.0 s) expires. The bridge must therefore never keep
re-sending a command that its own upstream (Nav2, the joystick) has stopped
refreshing: doing so resets ArduPilot's timeout on every send and the rover
never stops. This class returns zero as soon as the last command is older
than ``timeout_s``, or if none has arrived, or if it was not finite.

Time comes from an injected monotonic clock, never from ROS time, so a
/clock jump or use_sim_time cannot make a stale command look fresh.
"""

import math
import time
from collections.abc import Callable


class CmdVelWatchdog:
    """Hold the latest velocity command and report when it has gone stale."""

    def __init__(self, timeout_s: float, clock: Callable[[], float] = time.monotonic) -> None:
        """Create a watchdog that treats commands older than timeout_s seconds as stale.

        The injected clock must return monotonic time in seconds.
        Raise ValueError if timeout_s is nonpositive or nonfinite.
        """
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError(f'timeout_s must be a positive finite number, got {timeout_s!r}')
        self._timeout_s = timeout_s
        self._clock = clock
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._stamp: float | None = None

    @property
    def timeout_s(self) -> float:
        """Maximum command age, in seconds, before output is forced to zero."""
        return self._timeout_s

    def update(self, linear_x: float, angular_z: float) -> None:
        """Record linear_x (m/s) and angular_z (rad/s) with the current clock time.

        A nonfinite component leaves the previous command and its timestamp unchanged,
        so the old command ages out. Errors from the injected clock propagate.
        """
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            return
        self._linear_x = linear_x
        self._angular_z = angular_z
        self._stamp = self._clock()

    def current(self) -> tuple[float, float, bool]:
        """Return (linear_x in m/s, angular_z in rad/s, stale) without refreshing command age.

        With no command or an age greater than timeout_s, return (0.0, 0.0, True).
        At exactly timeout_s the command is still fresh. Errors from the injected clock propagate.
        """
        if self._stamp is None or self._clock() - self._stamp > self._timeout_s:
            return 0.0, 0.0, True
        return self._linear_x, self._angular_z, False
