import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, EqualsSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    # Package Paths
    devkit_pkg = get_package_share_directory('devkit_launch')
    ui_pkg = get_package_share_directory('devkit_ui')
    
    try:
        ublox_pkg = get_package_share_directory('ublox_dgnss')
    except:
        ublox_pkg = ''
        
    # Launch Configurations
    gps_type = LaunchConfiguration('gps_type', 
        default=EnvironmentVariable('GPS_TYPE_ROVER', default_value='ublox'))
    rover_port = LaunchConfiguration('rover_port', 
        default=EnvironmentVariable('GPS_PORT_ROVER', default_value='/dev/ttyACM0'))

    return LaunchDescription([
        DeclareLaunchArgument('gps_type', default_value=gps_type),
        DeclareLaunchArgument('rover_port', default_value=rover_port),

        # Option A: Ublox Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_pkg, 'launch', 'ublox_rover_hpposllh.launch.py')
            ),
            launch_arguments={
                'device': rover_port,
                'baudrate': '460800',  # Fixed: was 468000
                'frame_id': 'gps_link',
                'CFG_TMODE_MODE': '0'
            }.items(),
            # Crucial: This maps the Ublox topic name to what your UI expects
            # Ensure your UI node listens to /gpsfix
        ),

        # Option B: Septentrio Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(devkit_pkg, 'launch', 'gnss.launch.py')
            ),
            condition=IfCondition(EqualsSubstitution(gps_type, 'septentrio'))
        ),

        # Devkit Driver & UI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(devkit_pkg, 'launch', 'devkit_driver.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ui_pkg, 'launch', 'ui.launch.py'))
        ),
    ])
