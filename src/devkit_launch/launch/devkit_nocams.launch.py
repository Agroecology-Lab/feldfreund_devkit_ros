"""Main launch file for the devkit project."""
# pylint: disable=duplicate-code

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the complete devkit system."""
    pkg_dir = get_package_share_directory('devkit_launch')

    # Include the GNSS launch file
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gnss.launch.py')
        )
    )

    # Include the devkit driver launch file
    devkit_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'devkit_driver.launch.py')
        )
    )

    # Include the UI launch file
    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ui.launch.py')
        )
    )

    # Foxglove Bridge Node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',  # Allow external connections
            'tls': False,
            'compression': 'jpeg',  # Use JPEG compression for images
            'jpeg_quality': 75,
            'max_qos_depth': 10,
        }],
    )

    return LaunchDescription([
        foxglove_bridge,
        gnss_launch,
        devkit_driver_launch,
        ui_launch
    ])
