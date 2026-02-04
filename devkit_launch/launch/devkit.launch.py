"""Main launch file for the devkit project with dynamic GPS and Sowbot extension."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():
    pkg_dir = get_package_share_directory('devkit_launch')
    ui_pkg_dir = get_package_share_directory('devkit_ui')
    
    # 1. Grab environment settings from fixusb.py .env
    gps_type_env = EnvironmentVariable('GPS_TYPE', default_value='none')
    
    # 2. Define Launch Arguments
    arg_sowbot = DeclareLaunchArgument('sowbot', default_value='false', description='Launch Sowbot seeding system')
    arg_gps_type = DeclareLaunchArgument('gps_type', default_value=gps_type_env)

    # 3. GNSS Drivers (Conditional)
    septentrio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gnss.launch.py')),
        condition=LaunchConfigurationEquals('gps_type', 'septentrio')
    )

    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'ublox.launch.py')),
        condition=LaunchConfigurationEquals('gps_type', 'ublox')
    )

    # 4. Standard Components
    devkit_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'devkit_driver.launch.py'))
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'camera_system.launch.py'))
    )

    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ui_pkg_dir, 'launch', 'ui.launch.py'))
    )

    # =========================================================================
    # 5. SOWBOT EXTENSION 
    # This will optionally launch additional nodes
    # =========================================================================
    # sowbot_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sowbot_launch.py')),
    #     condition=IfCondition(LaunchConfiguration('sowbot'))
    # )

    return LaunchDescription([
        arg_gps_type,
        arg_sowbot,
        septentrio_launch,
        ublox_launch,
        devkit_driver_launch,
        camera_launch,
        ui_launch,
        # sowbot_launch, # Uncomment this once sowbot_launch.py is created
    ])
