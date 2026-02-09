"""Launch file for the complete camera system with conditional hardware loading."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition  # Required for conditional logic

def generate_launch_description():
    """Generate launch description for the complete camera system."""
    pkg_dir = get_package_share_directory('devkit_launch')

    # Read the hardware flags from the environment
    # Note: .env is loaded into the environment by manage.py
    use_axis = os.environ.get('AXIS_CAM_ENABLED', 'false').lower() == 'true'
    use_usb = os.environ.get('USB_CAM_ENABLED', 'false').lower() == 'true'

    # Include AXIS cameras only if flag is true
    axis_cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'axis_cameras.launch.py')
        ),
        condition=IfCondition(str(use_axis))
    )

    # Include USB camera only if flag is true
    usb_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'usb_camera.launch.py')
        ),
        condition=IfCondition(str(use_usb))
    )

    return LaunchDescription([
        axis_cameras_launch,
        usb_camera_launch,
    ])
