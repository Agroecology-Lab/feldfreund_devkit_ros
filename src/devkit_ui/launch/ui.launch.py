"""Launch file for the devkit UI node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the devkit UI."""
    ui_node = Node(
        package='devkit_ui',
        executable='ui_node',
        name='ui_node',
        output='screen',
        respawn=True,
        respawn_delay=5,
    )

    return LaunchDescription([
        ui_node,
    ])
