import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, FindExecutable


def generate_launch_description():
    pkg_name  = "agro_robot_sim"
    pkg_share = get_package_share_directory(pkg_name)

    # ── Launch arguments ──────────────────────────────────────────────────────
    x_arg     = DeclareLaunchArgument("x",     default_value="0.0")
    y_arg     = DeclareLaunchArgument("y",     default_value="0.0")
    z_arg     = DeclareLaunchArgument("z",     default_value="0.3")
    world_arg = DeclareLaunchArgument(
        "world", default_value="minha_fazenda.sdf",
        description="SDF world file name inside agro_robot_sim/worlds/",
    )
    urdf_arg  = DeclareLaunchArgument(
        "urdf", default_value="sowbot_01.xacro",
        description="URDF/xacro filename inside agro_robot_sim/urdf/",
    )

    # Inherit DISPLAY and XAUTHORITY from the shell so gz sim can open a window.
    # Without this, ExecuteProcess drops these variables and gz runs headless.
    display_env = {
        "DISPLAY":    os.environ.get("DISPLAY",    ":0"),
        "XAUTHORITY": os.environ.get("XAUTHORITY", ""),
    }

    # ── 1. Gazebo — direct call with display env ──────────────────────────────
    world_file = PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world")])
    gz_sim = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", "-r", world_file],
        name="gz_sim",
        output="screen",
        additional_env=display_env,
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
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

    # ── 4. Bridge (2 s after spawn exits) ────────────────────────────────────
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
