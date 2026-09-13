"""devkit_mavlink_bridge -- cmd_vel/localisation <-> ArduPilot Rover RTU.

See research/ardurover.md in Sowbot_Data for the full design and the
open items list. This node currently implements the outbound half
(cmd_vel -> SET_POSITION_TARGET_LOCAL_NED) only; the inbound half
(FusionCore pose -> GPS_INPUT, see modules/gps_input_handler.py) is
blocked on confirming FusionCore's output topic/type/rate first.

TODO (tracked in ardurover.md, not duplicated here -- update there,
not just in this comment, when one of these lands):
  1. Physical MAVLink port on the RTU Master Controller -- ask Robotriks.
  2. FusionCore output topic/type/rate -- needed for gps_input_handler.
  3. EKF-origin-on-first-boot behaviour when GPS_INPUT is the only GPS
     source -- may need an initial GPS_INPUT burst before GUIDED will
     accept velocity commands at all.
  5. Connection string / transport for pymavlink (serial device, baud)
     depends on item 1.
  6. RTU-side GPS1_TYPE=14 parameter change -- not this node's job, but
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

# GUID_TIMEOUT default is 3.0s (ArduPilot Rover) -- republish comfortably
# faster than that, not just on cmd_vel change, or the rover auto-stops
# even while the last command is still "current" from the bridge's POV.
_REPUBLISH_PERIOD_S = 0.5

_MAVLINK_ENDPOINT_ENV = 'MAVLINK_ENDPOINT'
_MAVLINK_ENDPOINT_PLACEHOLDER = 'TODO_SERIAL_DEVICE_HERE'
_MAVLINK_BAUD = 115200


class MavlinkBridgeNode(Node):
    """Bridges cmd_vel to an ArduPilot Rover RTU over MAVLink (GUIDED mode)."""

    def __init__(self):
        """Connect to MAVLink and register the ROS subscription and timer."""
        super().__init__('devkit_mavlink_bridge_node')

        self._last_twist = Twist()

        mavlink_endpoint = os.environ.get(_MAVLINK_ENDPOINT_ENV, '').strip()
        if not mavlink_endpoint or mavlink_endpoint.upper() == _MAVLINK_ENDPOINT_PLACEHOLDER:
            raise ValueError(
                f'{_MAVLINK_ENDPOINT_ENV} must be set to a valid MAVLink connection endpoint')

        self._mav = mavutil.mavlink_connection(
            mavlink_endpoint, baud=_MAVLINK_BAUD)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(_REPUBLISH_PERIOD_S, self._republish)

        self.get_logger().info(
            f'devkit_mavlink_bridge up, republishing every '
            f'{_REPUBLISH_PERIOD_S}s (GUID_TIMEOUT margin)')

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Cache the latest velocity command for periodic publishing."""
        self._last_twist = msg

    def _republish(self) -> None:
        """Translate and resend the latest velocity command over MAVLink."""
        fields = twist_to_position_target(
            linear_x=self._last_twist.linear.x,
            angular_z=self._last_twist.angular.z,
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
