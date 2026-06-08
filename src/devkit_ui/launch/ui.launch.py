"""Launch file for the devkit UI node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the devkit UI."""
    # sim flows down from devkit.launch.py. The UI node uses it to decide
    # whether to publish a fake /gnss/fix at the field datum (sim has no real
    # GPS publisher, but saving a topo map requires a finite, non-zero fix).
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
