import os
import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('devkit_launch'), 'config')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(config_directory, 'devkit.yaml')),
        DeclareLaunchArgument('mcu_port', default_value='/dev/ttyACM0'),
        Node(
            package='devkit_driver',
            executable='devkit_driver_node',
            name='controller',
            # This line forces the CLI argument to override the YAML file
            parameters=[LaunchConfiguration('config_file'), {'serial_port': LaunchConfiguration('mcu_port')}],
            respawn=True,
            respawn_delay=5,
        )
    ])
