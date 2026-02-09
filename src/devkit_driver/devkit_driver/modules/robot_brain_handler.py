from rclpy.node import Node
from rosys import background_tasks
from rosys.hardware import RobotBrain
from std_msgs.msg import Empty


class RobotBrainHandler:
    """Handler for the Zauberzeug Robot Brain."""

    def __init__(self, node: Node, robot_brain: RobotBrain):
        self.log = node.get_logger()
        self._robot_brain = robot_brain

        self._enable_sub = node.create_subscription(Empty, 'esp/enable', self._handle_enable, 10)
        self._disable_sub = node.create_subscription(Empty, 'esp/disable', self._handle_disable, 10)
        self._reset_sub = node.create_subscription(Empty, 'esp/reset', self._handle_reset, 10)
        self._restart_sub = node.create_subscription(Empty, 'esp/restart', self._handle_restart, 10)
        self._configure_sub = node.create_subscription(Empty, 'esp/configure', self._handle_configure, 10)

    def _handle_enable(self, _: Empty) -> None:
        background_tasks.create(self._robot_brain.enable_esp())

    def _handle_disable(self, _: Empty) -> None:
        background_tasks.create(self._robot_brain.disable_esp())

    def _handle_reset(self, _: Empty) -> None:
        background_tasks.create(self._robot_brain.reset_esp())

    def _handle_restart(self, _: Empty) -> None:
        background_tasks.create(self._robot_brain.restart())

    def _handle_configure(self, _: Empty) -> None:
        background_tasks.create(self._robot_brain.configure())
