"""CmdVelWatchdog must output zero for any command that is missing, old or not finite."""

import math
import unittest

from devkit_mavlink_bridge.modules.velocity_watchdog import CmdVelWatchdog


class FakeClock:
    """Manually advanced monotonic clock."""

    def __init__(self) -> None:
        """Start at an arbitrary non-zero time."""
        self.now = 1000.0

    def __call__(self) -> float:
        """Return the current fake time."""
        return self.now


class CmdVelWatchdogTest(unittest.TestCase):
    """Behaviour of the command-age watchdog."""

    def setUp(self) -> None:
        """Create a watchdog with a 0.5 s timeout on a fake clock."""
        self.clock = FakeClock()
        self.dog = CmdVelWatchdog(timeout_s=0.5, clock=self.clock)

    def test_no_command_yet_is_stale_zero(self):
        """Verify the output is zero and stale before any command has arrived."""
        self.assertEqual(self.dog.current(), (0.0, 0.0, True))

    def test_fresh_command_passes_through(self):
        """Verify a command younger than the timeout is returned unchanged."""
        self.dog.update(0.8, -0.3)
        self.clock.now += 0.4
        self.assertEqual(self.dog.current(), (0.8, -0.3, False))

    def test_command_at_exactly_the_timeout_is_still_fresh(self):
        """Verify the boundary: age == timeout is fresh, anything older is stale."""
        self.dog.update(0.8, 0.0)
        self.clock.now += 0.5
        self.assertFalse(self.dog.current()[2])

    def test_old_command_becomes_zero(self):
        """Verify a command older than the timeout is replaced by zero."""
        self.dog.update(0.8, -0.3)
        self.clock.now += 0.51
        self.assertEqual(self.dog.current(), (0.0, 0.0, True))

    def test_republishing_the_cached_value_does_not_refresh_it(self):
        """Verify reading the command repeatedly never extends its life."""
        self.dog.update(0.8, 0.0)
        for _ in range(10):
            self.clock.now += 0.1
            self.dog.current()
        self.assertEqual(self.dog.current(), (0.0, 0.0, True))

    def test_new_command_recovers_from_stale(self):
        """Verify motion resumes only when a new command arrives."""
        self.dog.update(0.8, 0.0)
        self.clock.now += 2.0
        self.assertTrue(self.dog.current()[2])
        self.dog.update(0.2, 0.1)
        self.assertEqual(self.dog.current(), (0.2, 0.1, False))

    def test_non_finite_command_is_ignored_and_old_one_ages_out(self):
        """Verify NaN or inf neither passes through nor refreshes the previous command."""
        self.dog.update(0.8, 0.0)
        self.clock.now += 0.3
        self.dog.update(math.nan, 0.0)
        self.dog.update(0.5, math.inf)
        self.assertEqual(self.dog.current(), (0.8, 0.0, False))
        self.clock.now += 0.3
        self.assertEqual(self.dog.current(), (0.0, 0.0, True))

    def test_invalid_timeout_is_rejected(self):
        """Verify a zero, negative or non-finite timeout cannot be configured."""
        for bad in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(timeout=bad), self.assertRaises(ValueError):
                CmdVelWatchdog(timeout_s=bad)


if __name__ == '__main__':
    unittest.main()
