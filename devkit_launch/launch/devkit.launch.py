import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    ublox_pkg = get_package_share_directory('ublox_dgnss_node')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')
    
    rover_port = os.getenv('GPS_PORT_ROVER', 'virtual')
    gps_type = os.getenv('GPS_TYPE_ROVER', 'ublox')
    mcu_port = os.getenv('MCU_PORT', 'virtual')

    # SAFETY CHECK: Only launch Ublox if the port is NOT virtual
    gps_enabled = PythonExpression(["'", rover_port, "' != 'virtual' and '", gps_type, "' == 'ublox'"])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),
        
        # Ublox Rover (Only starts if physical hardware is requested)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ublox_pkg, 'launch', 'ublox_rover_hpposllh.launch.py')),
            condition=IfCondition(gps_enabled),
            launch_arguments={'device': rover_port, 'baudrate': '460800'}.items(),
        ),

        # Devkit Driver (The Bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(devkit_launch_pkg, 'launch', 'devkit_driver.launch.py')),
            launch_arguments={'port': mcu_port}.items(),
        ),
    ])
