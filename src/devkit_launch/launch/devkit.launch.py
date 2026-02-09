"""Main launch file for the devkit project with dynamic GPS and Sowbot extension."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():
    # 1. Resolve Package Directories
    # Wrapping in a try/except provides a clearer error if the build hasn't installed the share directory yet
    try:
        pkg_dir = get_package_share_directory('devkit_launch')
        ui_pkg_dir = get_package_share_directory('devkit_ui')
    except Exception as e:
        return LaunchDescription([
            LogInfo(msg=f"CRITICAL: Could not find package share directory. Has the workspace been built? Error: {e}")
        ])

    # 2. Map environment variables from fixusb.py .env
    # This falls back to 'none' if hardware isn't detected or in virtual mode
    default_gps_type = EnvironmentVariable('GPS_TYPE_ROVER', default_value='none')
    
    # 3. Define Launch Arguments and Configurations
    sowbot_arg = LaunchConfiguration('sowbot')
    gps_type_arg = LaunchConfiguration('gps_type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sowbot', 
            default_value='false', 
            description='Launch Sowbot seeding system'
        ),
        DeclareLaunchArgument(
            'gps_type', 
            default_value=default_gps_type, 
            description='Type of GNSS: septentrio, ublox, or none'
        ),

        # 4. GNSS Drivers (Conditional based on resolved gps_type)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gnss.launch.py')),
            condition=LaunchConfigurationEquals('gps_type', 'septentrio')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'ublox.launch.py')),
            condition=LaunchConfigurationEquals('gps_type', 'ublox')
        ),

        # 5. Standard Components
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'devkit_driver.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'camera_system.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ui_pkg_dir, 'launch', 'ui.launch.py'))
        ),

        # 6. Sowbot Extension
        # Uncomment and create sowbot_launch.py when ready to integrate hardware actuators
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sowbot_launch.py')),
        #     condition=IfCondition(sowbot_arg)
        # ),
    ])
