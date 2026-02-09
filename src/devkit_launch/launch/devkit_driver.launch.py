import os
import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('devkit_launch'), 'config')

    # Prioritise MCU_PORT from .env, fallback to /dev/ttyACM0
    default_mcu_port = EnvironmentVariable('MCU_PORT', default_value='/dev/ttyACM0')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(config_directory, 'devkit.yaml')),
        DeclareLaunchArgument('mcu_port', default_value=default_mcu_port),
        Node(
            package='devkit_driver',
            executable='devkit_driver_node',
            name='controller',
            parameters=[LaunchConfiguration('config_file'), {'serial_port': LaunchConfiguration('mcu_port')}],
            respawn=True,
            respawn_delay=5,
        )
    ])
