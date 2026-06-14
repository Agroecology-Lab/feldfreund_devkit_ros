import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, TextSubstitution, FindExecutable


def generate_launch_description():
    pkg_name     = "agro_robot_sim"
    pkg_share    = get_package_share_directory(pkg_name)
    gz_pkg_share = get_package_share_directory("ros_gz_sim")

    # 0. launch arguments
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.3")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="minha_fazenda.sdf",
        description="SDF world file name inside agro_robot_sim/worlds/",
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value="sowbot_01.xacro",
        description="URDF/xacro filename inside agro_robot_sim/urdf/",
    )

    # 1. open gazebo
    world = LaunchConfiguration("world")
    world_file = PathJoinSubstitution([pkg_share, "worlds", world])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r "), world_file],
            "on_exit_shutdown": "True",
        }.items(),
    )

    # 2. robot_state_publisher
    # use_sim_time: False — see original comment.
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", LaunchConfiguration("urdf")])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", xacro_file]), value_type=str
                ),
                "use_sim_time": False,
            }
        ],
    )

    # 3. spawn robot
    #
    # WHY NOT -file xacro_file:
    #   gz sim create passes the file path to the URDF parser directly, which
    #   does not invoke xacro.  Variables like ${frame_len} are left unresolved
    #   and parsing fails with "Unable to parse component".
    #
    # WHY NOT -topic /robot_description:
    #   robot_state_publisher strips <gazebo> plugin blocks when publishing
    #   /robot_description (not valid URDF).  The spawned robot has no
    #   diff-drive, IMU, or NavSat plugins so /odom and /tf are never published.
    #
    # SOLUTION: run xacro as a pre-processing step and pipe the resolved URDF
    #   (including <gazebo> blocks) to gz sim create via stdin / -string.
    #   xacro preserves <gazebo> blocks; gz sim create -string sends the content
    #   to the URDF->SDF converter which handles them correctly.
    #
    # WHY POLL instead of TimerAction(period=8.0):
    #   maize.world takes 5-15 s to load depending on the host.  A fixed timer
    #   fires gz sim create before the /world/* services exist, which causes a
    #   silent no-op spawn.  Polling 'gz service -l' is event-driven and works
    #   regardless of host speed.
    spawn_entity = ExecuteProcess(
        cmd=[
            FindExecutable(name='bash'), '-c',
            [
                'echo "[spawn_robot] waiting for gz sim to be ready..."; '
                'until gz service -l 2>/dev/null | grep -q "/world/"; do sleep 2; done; '
                'echo "[spawn_robot] gz ready — spawning agro_robot"; '
                'URDF=$(xacro ', xacro_file, ') && '
                'ros2 run ros_gz_sim create'
                ' -name agro_robot'
                ' -string "$URDF"'
                ' -x ', LaunchConfiguration('x'),
                ' -y ', LaunchConfiguration('y'),
                ' -z ', LaunchConfiguration('z'),
            ],
        ],
        name='spawn_robot',
        output='screen',
    )

    # 4. topic bridge
    # Start 2 s after spawn_entity exits (event-driven, not a fixed timer).
    # The bridge subscribes to /model/agro_robot/... topics; those Gazebo topics
    # only exist once the model is in the world, so we must not start until
    # spawn_entity has completed successfully.
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")
    ros_gz_bridge = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        Node(
                            package="ros_gz_bridge",
                            executable="parameter_bridge",
                            name="ros_gz_bridge",
                            output="screen",
                            parameters=[
                                {"use_sim_time": True},
                                {"config_file": bridge_config},
                            ],
                        )
                    ],
                )
            ],
        )
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        world_arg,
        urdf_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])
