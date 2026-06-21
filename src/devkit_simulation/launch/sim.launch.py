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
from launch.substitutions import PathJoinSubstitution

GZ_BIN = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"


def generate_launch_description():
    pkg_name  = "devkit_simulation"
    pkg_share = get_package_share_directory(pkg_name)

    # Resolve paths eagerly at generate time — avoids substitution timing bugs
    # that cause gz to receive a blank world path or xacro to receive no input.
    models_dir  = os.path.join(os.path.dirname(pkg_share), "..", "..", "..", "models")
    models_dir  = os.path.realpath(models_dir)

    # ── Launch arguments ──────────────────────────────────────────────────────
    x_arg     = DeclareLaunchArgument("x", default_value="0.0")
    y_arg     = DeclareLaunchArgument("y", default_value="0.0")
    z_arg     = DeclareLaunchArgument("z", default_value="0.3")
    world_arg = DeclareLaunchArgument(
        "world", default_value="maize.world",
        description="SDF world file name inside devkit_simulation/worlds/",
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf", default_value="sowbot_01.xacro",
        description="URDF/xacro filename inside devkit_simulation/urdf/",
    )

    # ── Environment ───────────────────────────────────────────────────────────
    gz_env = {
        "DISPLAY": os.environ.get("DISPLAY", ":0"),
        "XAUTHORITY": os.environ.get("XAUTHORITY", ""),
        "GZ_SIM_RESOURCE_PATH": models_dir + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    }

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    # Use an eager world path string + LaunchConfiguration for the filename so
    # the world arg override still works, but the directory is resolved now.
    world_file = PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world")])
    gz_sim = ExecuteProcess(
        cmd=[GZ_BIN, "sim", "-r", world_file],
        name="gz_sim",
        output="screen",
        additional_env=gz_env,
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
    # Command([]) resolves PathJoinSubstitution correctly at node startup.
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
            "use_sim_time": True,
        }],
    )

    # ── 3. Spawn robot ────────────────────────────────────────────────────────
    # xacro_file_eager is resolved at generate time so the bash -c string
    # always receives a real path, not an unresolved substitution token.
    xacro_file_eager = os.path.join(pkg_share, "urdf", "sowbot_01.xacro")
    spawn_entity = ExecuteProcess(
        cmd=[
            "/bin/bash", "-c",
            (
                'echo "[spawn] waiting for gz sim..."; '
                f'until {GZ_BIN} service -l 2>/dev/null | grep -q "/world/"; do sleep 2; done; '
                'echo "[spawn] gz service ready — waiting 30s for GUI to finish init..."; '
                'sleep 30; '
                'echo "[spawn] spawning agro_robot"; '
                f'URDF=$(xacro {xacro_file_eager}) && '
                'ros2 run ros_gz_sim create'
                ' -name agro_robot'
                ' -string "$URDF"'
                ' -x 0.0 -y 0.0 -z 0.13'  # base_link_z=0.11 + 0.02 clearance
            ),
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
