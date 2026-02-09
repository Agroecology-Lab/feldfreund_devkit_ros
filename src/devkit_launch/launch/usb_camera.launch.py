"""Launch file for the USB camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for USB camera."""

    # Get the configuration file path
    config_file = os.path.join(
        get_package_share_directory('devkit_launch'),
        'config',
        'camera.yaml'
    )

    return LaunchDescription([
        # Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[config_file],
            remappings=[
                ('image_raw', '/devkit/camera/image_raw'),
                ('camera_info', '/devkit/camera/camera_info'),
            ]
        ),
    ])
