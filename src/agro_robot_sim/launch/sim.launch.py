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

    # ── Launch arguments ──────────────────────────────────────────────────────
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

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    world_file = PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world")])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r "), world_file],
            "on_exit_shutdown": "True",
        }.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
    # Command(["xacro ", xacro_file]) runs xacro at launch time and passes the
    # result as a proper ROS parameter — avoids all shell quoting issues that
    # break passing 23 kB of XML as a command-line argument.
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", LaunchConfiguration("urdf")])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", xacro_file]), value_type=str
            ),
            "use_sim_time": False,
        }],
    )

    # ── 3. Spawn robot ────────────────────────────────────────────────────────
    # WHY -string and not -topic /robot_description:
    #   robot_state_publisher strips <gazebo> plugin blocks before publishing.
    #   Spawning from that topic gives a robot with no diff-drive / IMU / NavSat.
    #
    # WHY poll instead of TimerAction:
    #   World load time varies (5–15 s on slow hosts). Polling gz service -l is
    #   event-driven and fires exactly when the world services exist.
    spawn_entity = ExecuteProcess(
        cmd=[
            FindExecutable(name="bash"), "-c",
            [
                'echo "[spawn] waiting for gz sim..."; '
                'until gz service -l 2>/dev/null | grep -q "/world/"; do sleep 2; done; '
                'echo "[spawn] gz ready — spawning agro_robot"; '
                'URDF=$(xacro ', xacro_file, ') && '
                'ros2 run ros_gz_sim create'
                ' -name agro_robot'
                ' -string "$URDF"'
                ' -x ', LaunchConfiguration("x"),
                ' -y ', LaunchConfiguration("y"),
                ' -z ', LaunchConfiguration("z"),
            ],
        ],
        name="spawn_robot",
        output="screen",
    )

    # ── 4. ros_gz_bridge (starts 2 s after spawn exits) ──────────────────────
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
        x_arg, y_arg, z_arg,
        world_arg, urdf_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])
