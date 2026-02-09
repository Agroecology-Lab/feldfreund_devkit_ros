import rosys
from rclpy.node import Node
from rosys import background_tasks
from rosys.hardware import EStop, EStopHardware
from std_msgs.msg import Bool

from ..qos import SAFETY_QOS


class EStopHandler:
    """Handle the estop."""
    FRONT_ID = 'front'
    BACK_ID = 'back'

    def __init__(self, node: Node, estop: EStop):
        self.log = node.get_logger()
        self._estop = estop

        self.subscription = node.create_subscription(Bool, 'estop/soft', self.soft_estop_callback, 10)
        self.estop_front_publisher = node.create_publisher(Bool, 'estop/front', SAFETY_QOS)
        self.estop_back_publisher = node.create_publisher(Bool, 'estop/back', SAFETY_QOS)

        self._estop.ESTOP_TRIGGERED.subscribe(self._handle_estop_triggered)
        self._estop.ESTOP_RELEASED.subscribe(self._handle_estop_released)
        rosys.on_startup(self._check_on_startup)

    def _check_on_startup(self) -> None:
        if not isinstance(self._estop, EStopHardware):
            return
        for name in self._estop.pins.keys():
            if name in self._estop.active_estops:
                self._handle_estop_triggered(name)
            else:
                self._handle_estop_released(name)

    def _handle_estop_triggered(self, name: str) -> None:
        if name == self.FRONT_ID:
            self.estop_front_publisher.publish(Bool(data=True))
        elif name == self.BACK_ID:
            self.estop_back_publisher.publish(Bool(data=True))

    def _handle_estop_released(self, name: str) -> None:
        if name == self.FRONT_ID:
            self.estop_front_publisher.publish(Bool(data=False))
        elif name == self.BACK_ID:
            self.estop_back_publisher.publish(Bool(data=False))

    def soft_estop_callback(self, msg: Bool):
        """Implement a callback for the estop."""
        background_tasks.create(self._estop.set_soft_estop(msg.data))
