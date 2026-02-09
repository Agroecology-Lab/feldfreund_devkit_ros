import rosys
from rclpy.node import Node
from rosys.hardware import Bumper, EStop
from std_msgs.msg import Bool

from ..qos import SAFETY_QOS


class BumperHandler:
    """Handle the bumper states from core data."""

    def __init__(self, node: Node, bumper: Bumper, estop: EStop):
        self.log = node.get_logger()
        self._bumper = bumper
        self._estop = estop

        self._pub_front_top = node.create_publisher(Bool, 'bumper/front_top', SAFETY_QOS)
        self._pub_front_bottom = node.create_publisher(Bool, 'bumper/front_bottom', SAFETY_QOS)
        self._pub_back = node.create_publisher(Bool, 'bumper/back', SAFETY_QOS)

        self._bumper.BUMPER_TRIGGERED.subscribe(self._handle_bumper_triggered)
        self._bumper.BUMPER_RELEASED.subscribe(self._handle_bumper_released)
        rosys.on_startup(self._check_on_startup)

    def _check_on_startup(self) -> None:
        for name in self._bumper.active_bumpers:
            self._handle_bumper_triggered(name)

    def _handle_bumper_triggered(self, bumper_name: str) -> None:
        """Handle bumper triggered event."""
        if bumper_name == 'front_top':
            self._pub_front_top.publish(Bool(data=True))
        elif bumper_name == 'front_bottom':
            self._pub_front_bottom.publish(Bool(data=True))
        elif bumper_name == 'back':
            self._pub_back.publish(Bool(data=True))

    def _handle_bumper_released(self, bumper_name: str) -> None:
        """Handle bumper released event."""
        if bumper_name == 'front_top':
            self._pub_front_top.publish(Bool(data=False))
        elif bumper_name == 'front_bottom':
            self._pub_front_bottom.publish(Bool(data=False))
        elif bumper_name == 'back':
            self._pub_back.publish(Bool(data=False))
