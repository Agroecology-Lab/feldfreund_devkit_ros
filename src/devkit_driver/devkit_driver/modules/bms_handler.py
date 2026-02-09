from rclpy.node import Node
from rosys.hardware import Bms
from rosys.hardware.bms_state import BmsState
from sensor_msgs.msg import BatteryState


class BMSHandler:
    """Handle the odometry."""

    def __init__(self, node: Node, bms: Bms):
        self.log = node.get_logger()
        self.node = node
        self._publisher = node.create_publisher(BatteryState, 'battery_state', 10)
        self._bms = bms
        self._bms.STATE_UPDATED.subscribe(self._handle_bms_update)

    def _handle_bms_update(self, state: BmsState) -> None:
        """Handle BMS update event."""
        if None in (state.voltage, state.current, state.percentage, state.temperature):
            self.log.warning('BMS state has None values, skipping publish')
            return
        message = self._state_to_ros_message(state)
        self._publisher.publish(message)

    def _state_to_ros_message(self, state: BmsState) -> BatteryState:
        """Convert BMS state to ROS BatteryState."""
        assert state.voltage is not None
        assert state.current is not None
        assert state.percentage is not None
        assert state.temperature is not None
        message = BatteryState()
        # TODO: rosys time to ros time
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.voltage = float(state.voltage)
        message.current = float(state.current)
        message.percentage = float(state.percentage) / 100.0
        message.temperature = float(state.temperature)
        return message
