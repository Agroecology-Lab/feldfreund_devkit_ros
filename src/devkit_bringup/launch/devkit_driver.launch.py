from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='virtual',
        description='Serial port for the MCU (e.g. /dev/ttyUSB0 or virtual)',
    )

    devkit_driver_node = Node(
        package='devkit_driver',
        executable='devkit_driver_node',
        name='devkit_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
        }],
    )

    return LaunchDescription([
        port_arg,
        devkit_driver_node,
    ])
