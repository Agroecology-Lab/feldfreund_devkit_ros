"""Launch file for the devkit project in simulation mode (driver + UI + Foxglove)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('devkit_bringup')

    devkit_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'devkit_driver.launch.py')
        ),
        launch_arguments={'simulation': 'true'}.items()
    )

    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ui.launch.py')
        )
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'compression': 'jpeg',
            'jpeg_quality': 75,
            'max_qos_depth': 10,
        }],
    )

    return LaunchDescription([
        devkit_driver_launch,
        ui_launch,
        foxglove_bridge,
    ])
