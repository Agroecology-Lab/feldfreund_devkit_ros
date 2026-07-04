"""Launch file for the devkit UI node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the devkit UI."""
    # sim flows down from devkit.launch.py. The UI node uses it to decide
    # whether to publish a fake fix on /gnss/fix_sim_shim at the field
    # datum, purely as a cold-start fallback for the UI's own topo-map save
    # path (see ui_node._publish_fake_gps / store_fake_gps). This used to
    # publish onto /gnss/fix itself — the same topic ros_gz_bridge bridges
    # Gazebo's real navsat sensor onto — and rely on a discovery-timing
    # backoff to yield to the real fix. That race could be lost once during
    # DDS discovery, letting the placeholder datum reach fusioncore and
    # anchor its filter ~53m from the real field, silently rejecting every
    # subsequent real fix as an outlier. Moving the shim to its own topic
    # removes the race entirely: fusioncore never subscribes to it.
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true = sim mode; UI publishes a fake GPS fix.',
    )
    sim = LaunchConfiguration('sim')
    # Normalise to a real bool for the ROS parameter.
    sim_bool = PythonExpression(["'", sim, "' == 'true'"])

    ui_node = Node(
        package='devkit_ui',
        executable='ui_node',
        name='ui_node',
        output='screen',
        respawn=True,
        respawn_delay=5,
        parameters=[{'sim': sim_bool}],
    )

    return LaunchDescription([
        sim_arg,
        ui_node,
    ])
