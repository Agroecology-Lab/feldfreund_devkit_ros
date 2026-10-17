"""devkit_mavlink_bridge -- cmd_vel/localisation <-> ArduPilot Rover on LEVIA-H7.

See research/ardurover.md in Sowbot_Data for the full design and the
open items list. This node currently implements the outbound half
(cmd_vel -> SET_POSITION_TARGET_LOCAL_NED) only; the inbound half
(FusionCore pose -> GPS_INPUT, see modules/gps_input_handler.py) is
blocked on confirming FusionCore's output topic/type/rate first.

TODO (tracked in ardurover.md, not duplicated here -- update there,
not just in this comment, when one of these lands):
  1. Which LEVIA-H7 UART carries MAVLink (safety_roadmap.md O31).
  2. FusionCore output topic/type/rate -- needed for gps_input_handler.
  3. EKF-origin-on-first-boot behaviour when GPS_INPUT is the only GPS
     source -- may need an initial GPS_INPUT burst before GUIDED will
     accept velocity commands at all.
  5. Connection string / transport for pymavlink (serial device, baud)
     depends on item 1.
  6. Flight-controller GPS1_TYPE=14 parameter change -- not this node's job, but
     nothing here works until it's set.
"""

import os
import time

import rclpy
from geometry_msgs.msg import Twist
from pymavlink import mavutil
from rclpy.node import Node

from devkit_mavlink_bridge.modules.position_target_handler import (
    twist_to_position_target,
)
from devkit_mavlink_bridge.modules.velocity_watchdog import CmdVelWatchdog

# ArduPilot Rover's GUID_TIMEOUT (default 3.0 s) only stops the rover if the
# bridge stops sending. The bridge therefore must not keep re-sending a command
# its upstream has stopped refreshing: after CMD_VEL_TIMEOUT_S without a new
# cmd_vel it sends an explicit zero velocity instead (see CmdVelWatchdog).
# At the 1.6 m/s top speed the 3 s default alone is ~4.8 m of travel. Lowering
# GUID_TIMEOUT on the flight controller (try ~1 s) is a second layer; measure
# the real stopping distance, these are starting values only.
_DEFAULT_REPUBLISH_PERIOD_S = 0.1
_DEFAULT_CMD_VEL_TIMEOUT_S = 0.5

_MAVLINK_ENDPOINT_ENV = 'MAVLINK_ENDPOINT'
_MAVLINK_ENDPOINT_PLACEHOLDER = 'TODO_SERIAL_DEVICE_HERE'
_MAVLINK_BAUD_ENV = 'MAVLINK_BAUD'
_DEFAULT_MAVLINK_BAUD = 115200  # must match SERIALn_BAUD on the flight controller


class MavlinkBridgeNode(Node):
    """Bridges cmd_vel to an ArduPilot Rover RTU over MAVLink (GUIDED mode)."""

    def __init__(self):
        """Connect to MAVLink and register the ROS subscription and timer.

        ROS parameters republish_period_s and cmd_vel_timeout_s are in seconds, defaulting
        to 0.1 and 0.5. MAVLINK_ENDPOINT selects the connection; MAVLINK_BAUD defaults to 115200.
        Raise ValueError for a nonpositive republish period, a nonpositive or nonfinite command
        timeout, a missing or placeholder endpoint, or a baud value that cannot be parsed as an int.
        MAVLink connection errors propagate.
        """
        super().__init__('devkit_mavlink_bridge_node')

        self.declare_parameter('republish_period_s', _DEFAULT_REPUBLISH_PERIOD_S)
        self.declare_parameter('cmd_vel_timeout_s', _DEFAULT_CMD_VEL_TIMEOUT_S)
        republish_period_s = float(self.get_parameter('republish_period_s').value)
        cmd_vel_timeout_s = float(self.get_parameter('cmd_vel_timeout_s').value)
        if republish_period_s <= 0.0:
            raise ValueError('republish_period_s must be positive')
        # Raises ValueError for a non-positive or non-finite timeout.
        self._watchdog = CmdVelWatchdog(cmd_vel_timeout_s)
        self._was_stale = True

        mavlink_endpoint = os.environ.get(_MAVLINK_ENDPOINT_ENV, '').strip()
        if not mavlink_endpoint or mavlink_endpoint.upper() == _MAVLINK_ENDPOINT_PLACEHOLDER:
            raise ValueError(
                f'{_MAVLINK_ENDPOINT_ENV} must be set to a valid MAVLink connection endpoint')
        baud = int(os.environ.get(_MAVLINK_BAUD_ENV, _DEFAULT_MAVLINK_BAUD))

        self._mav = mavutil.mavlink_connection(mavlink_endpoint, baud=baud)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(republish_period_s, self._republish)

        self.get_logger().info(
            f'devkit_mavlink_bridge up on {mavlink_endpoint} @ {baud} baud: sending every '
            f'{republish_period_s}s, zero velocity once cmd_vel is older than {cmd_vel_timeout_s}s')

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Record linear.x (m/s), angular.z (rad/s), and their monotonic arrival time.

        If either component is nonfinite, leave the previous command and arrival time unchanged.
        """
        self._watchdog.update(msg.linear.x, msg.angular.z)

    def _republish(self) -> None:
        """Send the current velocity target, or zero if no command exists or it has gone stale.

        MAVLink send errors propagate.
        """
        linear_x, angular_z, stale = self._watchdog.current()
        if stale != self._was_stale:
            if stale:
                self.get_logger().warn(
                    f'cmd_vel older than {self._watchdog.timeout_s}s or absent: commanding zero velocity')
            else:
                self.get_logger().info('cmd_vel fresh again: forwarding commands')
            self._was_stale = stale
        fields = twist_to_position_target(
            linear_x=linear_x,
            angular_z=angular_z,
            time_boot_ms=(time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF,
        )
        self._mav.mav.set_position_target_local_ned_send(**fields)


def main(args=None):
    """Run the MAVLink bridge node until ROS shuts down."""
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
