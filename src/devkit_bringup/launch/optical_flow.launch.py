"""Launch the optical flow driver with this project's parameter overrides.

Follows optical_flow_ros/launch/optical_flow_launch.py (Apache-2.0, Aditya Kamath; revision pinned by
OPTICAL_FLOW_ROS_COMMIT in docker/Dockerfile), but loads config/optical_flow.yaml after the package's own
sensor_params.yaml. That keeps odom -> base_link owned by FusionCore without patching the clone.

optical_flow_ros is built from source in the image and rosdep cannot resolve it, so it is deliberately not an
exec_depend of this package.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Return a launch description that configures and activates the optical flow node.

    Load project overrides after the driver's sensor parameters and remap odom
    to flow_odom. Request activation only on configuring -> inactive transitions,
    leaving deliberate deactivation intact.

    Raise PackageNotFoundError if optical_flow_ros or devkit_bringup cannot be
    found in the ament index.
    """
    node = LifecycleNode(
        package='optical_flow_ros',
        executable='optical_flow_publisher',
        name='optical_flow',
        namespace='',  # NOTE: The parameter files and the remapping below are keyed on this name.
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('optical_flow_ros'), 'config', 'sensor_params.yaml'),
            # NOTE: Later files win, so the overrides must stay last.
            os.path.join(get_package_share_directory('devkit_bringup'), 'config', 'optical_flow.yaml'),
        ],
        remappings=[('odom', 'flow_odom')],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    # NOTE: Only react to configuring -> inactive, so a deliberate deactivate is not undone.
    activate_after_configure = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    return LaunchDescription([node, configure, activate_after_configure])
