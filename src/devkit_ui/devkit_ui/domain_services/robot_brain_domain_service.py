from __future__ import annotations

from std_msgs.msg import Empty

from devkit_ui.ros_gateway import RosGateway

ROBOT_BRAIN_COMMANDS = ('enable', 'disable', 'reset', 'restart', 'configure')


class RobotBrainDomainService:
    """Publish control commands to the robot brain (the ESP32 running Lizard).

    The topics are esp/enable, esp/disable, esp/reset, esp/restart and esp/configure;
    robot_brain_handler in devkit_driver subscribes to them.
    """

    def __init__(self, ros: RosGateway) -> None:
        """Create one empty-message publisher for each supported robot brain command."""
        self._pubs = {
            command: ros.create_publisher(Empty, f'esp/{command}', 1)
            for command in ROBOT_BRAIN_COMMANDS
        }

    def send(self, command: str) -> None:
        """Publish an empty message on esp/<command>.

        Raises:
            ValueError: command is not one of ROBOT_BRAIN_COMMANDS.
        """
        if command not in self._pubs:
            raise ValueError(f'unknown robot brain command: {command!r}')
        self._pubs[command].publish(Empty())
