# neo.launch.py
# =============
# Launch file for the Neo SBC.
# Runs the USB camera (usb_cam) + crop_row_node (row following + /cmd_vel).
#
# Delegates to crop_row_nav.launch.py in sowbot_row_follow — that package
# owns the node definitions and default params (crop_row_params.yaml), and
# now also brings up usb_cam so the camera publisher and crop_row_node
# subscriber share one launch session / DDS graph.
#
# camera_index, video_device and use_camera are launch args in
# crop_row_nav.launch.py and can be overridden on the command line. Other node
# parameters (linear_vel, max_omega, camera_height_m etc.) live in
# crop_row_params.yaml — edit that file for permanent changes, or override via
# ros2 param set at runtime.
#
# Limbic-side complement: row_follow.launch.py (limbic_row_follow action
# server) is included in devkit.launch.py under real_condition.
#
# Usage:
#   python manage.py neo                       # defaults from crop_row_params.yaml
#   python manage.py neo camera_index:=2       # alternate OpenCV index
#   python manage.py neo video_device:=/dev/video2   # alternate V4L2 device
#   python manage.py neo use_camera:=false     # subscribe only (external camera)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    sowbot_pkg = get_package_share_directory('sowbot_row_follow')
    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0',
                              description='OpenCV VideoCapture index'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='V4L2 device node for the USB camera'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='Launch usb_cam here; set false for an external camera source'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sowbot_pkg, 'launch', 'crop_row_nav.launch.py')
            ),
            launch_arguments={
                'camera_index': LaunchConfiguration('camera_index'),
                'video_device': LaunchConfiguration('video_device'),
                'use_camera': LaunchConfiguration('use_camera'),
            }.items(),
        ),
    ])
