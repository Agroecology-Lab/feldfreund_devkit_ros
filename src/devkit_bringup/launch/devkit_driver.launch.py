from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='virtual',
        description='Serial port for the MCU (e.g. /dev/ttyUSB0 or virtual)',
    )

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode (true/false)',
    )

    simulation = LaunchConfiguration('simulation')

    set_simulation_env = SetEnvironmentVariable(
        'FELDFREUND_SIMULATION',
        simulation,
    )

    devkit_driver_node = Node(
        package='devkit_driver',
        executable='devkit_driver_node',
        name='devkit_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'use_sim_time': simulation,
        }],
    )

    return LaunchDescription([
        port_arg,
        simulation_arg,
        set_simulation_env,
        devkit_driver_node,
    ])
