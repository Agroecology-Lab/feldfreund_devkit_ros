"""Main launch file for the devkit project with dynamic GPS and Sowbot extension."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    pkg_dir = get_package_share_directory('devkit_launch')
    ui_pkg_dir = get_package_share_directory('devkit_ui')
    
    # 1. Map environment variables from fixusb.py .env (with Rover suffix)
    # This falls back to 'none' if fixusb.py hasn't run or is in virtual mode
    default_gps_type = EnvironmentVariable('GPS_TYPE_ROVER', default_value='none')
    
    # 2. Define Launch Arguments
    # Using LaunchConfiguration allows manage.py to override these via CLI
    return LaunchDescription([
        DeclareLaunchArgument('sowbot', default_value='false', description='Launch Sowbot seeding system'),
        DeclareLaunchArgument('gps_type', default_value=default_gps_type, description='Type of GNSS: septentrio, ublox, or none'),

        # 3. GNSS Drivers (Conditional based on resolved gps_type)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gnss.launch.py')),
            condition=LaunchConfigurationEquals('gps_type', 'septentrio')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'ublox.launch.py')),
            condition=LaunchConfigurationEquals('gps_type', 'ublox')
        ),

        # 4. Standard Components
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'devkit_driver.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'camera_system.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ui_pkg_dir, 'launch', 'ui.launch.py'))
        ),

        # NOTE: SOWBOT EXTENSION 
        # Uncomment and create sowbot_launch.py when ready to integrate hardware actuators
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sowbot_launch.py')),
        #     condition=IfCondition(LaunchConfiguration('sowbot'))
        # ),
    ])
