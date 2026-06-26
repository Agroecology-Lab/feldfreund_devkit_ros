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
            # Republish fixed joints on /tf at 50 Hz instead of one-shot
            # /tf_static. Without this, Nav2 nodes that join after sim startup
            # miss the transient-local /tf_static delivery (stamped at t=0 sim
            # time, appears ancient by the time Nav2 is alive) and the
            # base_footprint -> base_link transform is never seen.
            "publish_frequency": 50.0,
            "ignore_timestamp": True,
        }],
    )

    # ── 3. Spawn robot ────────────────────────────────────────────────────────
    # DEVKIT_URDF is set by sowbot_sim.launch.py via SetEnvironmentVariable
    # before this subprocess starts, so the bash variable resolves at runtime
    # to the model selected in the UI.  os.environ.get() is NOT used here
    # because SetEnvironmentVariable is a substitution that hasn't evaluated
    # yet when generate_launch_description() runs — it would always be empty.
    urdf_dir = os.path.join(pkg_share, "urdf")
    spawn_entity = ExecuteProcess(
        cmd=[
            "/bin/bash", "-c",
            (
                'echo "[spawn] waiting for gz sim..."; '
                f'until {GZ_BIN} service -l 2>/dev/null | grep -q "/world/"; do sleep 2; done; '
                'echo "[spawn] gz service ready — waiting 30s for GUI to finish init..."; '
                'sleep 30; '
                'echo "[spawn] spawning agro_robot"; '
                f'_URDF="${{DEVKIT_URDF:-sowbot_01.xacro}}"; '
                f'echo "[spawn] using URDF: {urdf_dir}/$_URDF"; '
                f'URDF=$(xacro {urdf_dir}/$_URDF) && '
                'ros2 run ros_gz_sim create'
                ' -name agro_robot'
                ' -string "$URDF"'
                ' -x 0.0 -y 0.0 -z 0.01'  # base_footprint on ground; base_link raised by wheel_radius via base_footprint_joint
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
