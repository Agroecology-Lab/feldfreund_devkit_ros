from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosys import background_tasks
from rosys.hardware import Wheels


class TwistHandler:
    """Relate ROS cmd_vel messages to RoSys wheel commands."""

    def __init__(self, node: Node, wheels: Wheels):
        self.log = node.get_logger()
        self._wheels = wheels
        self._cmd_subscription = node.create_subscription(Twist, 'cmd_vel', self.handle_twist, 10)

    def handle_twist(self, cmd_msg: Twist) -> None:
        """Implement callback for cmd_vel message."""
        background_tasks.create(self._wheels.drive(cmd_msg.linear.x, cmd_msg.angular.z))
