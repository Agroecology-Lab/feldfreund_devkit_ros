# neo.launch.py
# =============
# Launch file for the Neo SBC.
# Runs crop_row_node (camera + row following + /cmd_vel publisher).
#
# Delegates to crop_row_nav.launch.py in sowbot_row_follow — that package
# owns the node definition and default params (crop_row_params.yaml).
#
# camera_index is a proper launch arg in crop_row_nav.launch.py and can be
# overridden on the command line. Other node parameters (linear_vel, max_omega,
# camera_height_m etc.) live in crop_row_params.yaml — edit that file for
# permanent changes, or override via ros2 param set at runtime.
#
# Limbic-side complement: row_follow.launch.py (limbic_row_follow action
# server) is included in devkit.launch.py under real_condition.
#
# Usage:
#   python manage.py neo                    # defaults from crop_row_params.yaml
#   python manage.py neo camera_index:=2   # alternate camera device

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sowbot_pkg, 'launch', 'crop_row_nav.launch.py')
            ),
            launch_arguments={
                'camera_index': LaunchConfiguration('camera_index'),
            }.items(),
        ),
    ])
