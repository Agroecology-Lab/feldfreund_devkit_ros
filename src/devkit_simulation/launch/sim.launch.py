import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

GZ_BIN = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"


def generate_launch_description():
    """
    Build the Gazebo launch with robot spawning and ROS bridges, optionally headless.

    Return a launch description for Gazebo, robot_state_publisher, robot
    spawning, and a ROS bridge scheduled two seconds after the spawn process
    exits with status 0. A non-zero spawn status logs the failure and starts no
    bridge from this launch description.

    Pass ``use_camera``, ``camera_width``, ``camera_height``, and ``camera_rate``
    to both xacro invocations. For the default sowbot_01.xacro, these enable a
    320-by-240-pixel camera at 10 Hz; ``use_camera:=false`` omits the sensor.
    Spawn at the coordinates in /workspace/spawn_pose.txt, falling back to
    (0, 0, 1) meters if the file is absent; the declared x/y/z arguments do not
    control the spawn position. While Gazebo reports no world, the spawn
    process checks the SPAWN_GZ_TIMEOUT_S deadline (default 180 seconds) between
    two-second polling sleeps and exits with status 1 on timeout. After a world
    is reported, it waits another 30 seconds before spawning; the deadline does
    not bound that delay or the spawn command.

    Raise PackageNotFoundError if devkit_simulation cannot be found in the
    ament index.
    """
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
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="true: run gz sim server-only (no GUI, no X display needed)",
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf", default_value="sowbot_01.xacro",
        description="URDF/xacro filename inside devkit_simulation/urdf/",
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Enable the simulated camera sensor',
    )
    camera_width_arg = DeclareLaunchArgument(
        'camera_width', default_value='320',
        description='Simulated camera image width in pixels',
    )
    camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='240',
        description='Simulated camera image height in pixels',
    )
    camera_rate_arg = DeclareLaunchArgument(
        'camera_rate', default_value='10',
        description='Simulated camera update rate in Hz',
    )

    # ── Environment ───────────────────────────────────────────────────────────
    gz_env = {
        "DISPLAY": os.environ.get("DISPLAY", ":0"),
        "XAUTHORITY": os.environ.get("XAUTHORITY", ""),
        "GZ_SIM_RESOURCE_PATH": models_dir + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    }

    # Headless on boards whose GPU can't give Ogre2 a GL 3.3 context (e.g.
    # aarch64 SBCs with Mali): force Mesa llvmpipe on a private Xvfb display.
    # LIBGL_ALWAYS_SOFTWARE is ignored by Mesa's EGL when the render node is
    # visible, so hide /dev/dri from gz when mount namespaces are permitted.
    gz_env_headless = {
        **gz_env,
        "DISPLAY": ":99",
        "XAUTHORITY": "",
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "GALLIUM_DRIVER": "llvmpipe",
        "MESA_GL_VERSION_OVERRIDE": "3.3",
        "MESA_GLSL_VERSION_OVERRIDE": "330",
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
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    # Headless: server-only (-s, no GUI) rendered in software.
    #  1. ensure display :99 accepts connections before starting Gazebo
    #  2. optionally hide hardware render nodes in a private mount namespace
    #  3. exec gz sim without --headless-rendering (that flag forces the EGL
    #     device path, which picks the hardware node and segfaults in Mesa)
    gz_sim_headless = ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'command -v xdpyinfo >/dev/null || '
                '{ echo "[gz] Install x11-utils for display readiness checks" >&2; exit 1; }; '
                'if ! timeout 1s xdpyinfo -display :99 >/dev/null 2>&1; then '
                'Xvfb :99 -screen 0 1280x720x24 -nolisten tcp >/dev/null 2>&1 & '
                'deadline=$((SECONDS + 10)); '
                'until timeout 1s xdpyinfo -display :99 >/dev/null 2>&1; do '
                'if (( SECONDS >= deadline )); then '
                'echo "[gz] Display :99 did not become ready" >&2; exit 1; fi; '
                'sleep 0.2; done; fi; '
                'if [ -d /dev/dri ] && unshare -m true 2>/dev/null; then '
                'exec unshare -m bash -c '
                "'mount -t tmpfs tmpfs /dev/dri || exit 1; exec \"$@\"' "
                f'bash "{GZ_BIN}" sim -r -s "$1"; fi; '
                'if [ -d /dev/dri ]; then '
                'echo "[gz] Mount namespace unavailable; using software rendering '
                'with /dev/dri visible" >&2; fi; '
                f'exec "{GZ_BIN}" sim -r -s "$1"'
            ),
            'gz_sim_headless', world_file,
        ],
        name="gz_sim",
        output="screen",
        additional_env=gz_env_headless,
        condition=IfCondition(LaunchConfiguration("headless")),
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
                Command([
                    'xacro "', xacro_file,
                    '" use_camera:=', LaunchConfiguration('use_camera'),
                    ' camera_width:=', LaunchConfiguration('camera_width'),
                    ' camera_height:=', LaunchConfiguration('camera_height'),
                    ' camera_rate:=', LaunchConfiguration('camera_rate'),
                ]), value_type=str
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
                # Bounded wait: without a deadline a gz that never comes up
                # leaves this process looping forever, so the spawn never exits
                # and nothing downstream is released or reported. Exiting
                # non-zero lets the bridge gate below fail closed.
                'GZ_WAIT_S="${SPAWN_GZ_TIMEOUT_S:-180}"; '
                'deadline=$((SECONDS + GZ_WAIT_S)); '
                f'until {GZ_BIN} service -l 2>/dev/null | grep -q "/world/"; do '
                'if (( SECONDS >= deadline )); then '
                'echo "[spawn] gz sim reported no world within ${GZ_WAIT_S}s" >&2; '
                'exit 1; fi; '
                'sleep 2; done; '
                'echo "[spawn] gz service ready — waiting 30s for GUI to finish init..."; '
                'sleep 30; '
                'echo "[spawn] spawning agro_robot"; '
                f'_URDF="${{DEVKIT_URDF:-sowbot_01.xacro}}"; '
                f'echo "[spawn] using URDF: {urdf_dir}/$_URDF"; '
                # Spawn pose comes from topo_to_forest3d.py's HOME-node +
                # terrain_offset calculation, not a hardcoded origin — the
                # world's terrain/crops are shifted by terrain_offset, so a
                # fixed (0,0) spawn drifts off the topo HOME position once
                # that offset is non-zero. Falls back to legacy (0,0,0.01)
                # if the file is missing (e.g. non-sim/manual runs).
                'SPAWN_FILE="/workspace/spawn_pose.txt"; '
                'if [ -f "$SPAWN_FILE" ]; then '
                'read -r SPAWN_X SPAWN_Y SPAWN_Z < "$SPAWN_FILE"; '
                'echo "[spawn] using spawn pose from $SPAWN_FILE: $SPAWN_X $SPAWN_Y $SPAWN_Z"; '
                'else '
                # 1.0m drop height, not a ground-level offset: terrain
                # elevation at (0,0) varies with each generated world, so a
                # near-zero Z (the old 0.01 default) spawns the robot
                # partially embedded in non-flat terrain, tanking the physics
                # real-time factor and producing bad wheel contact. Dropping
                # from 1.0m and letting gravity settle onto the mesh is
                # robust regardless of terrain shape at this fallback point.
                'SPAWN_X=0.0; SPAWN_Y=0.0; SPAWN_Z=1.0; '
                'echo "[spawn] WARNING: $SPAWN_FILE not found — falling back to (0,0,1.0), robot will drop onto terrain"; '
                'fi; '
                f'URDF=$(xacro "{urdf_dir}/$_URDF" '
                '"use_camera:=$1" "camera_width:=$2" '
                '"camera_height:=$3" "camera_rate:=$4") && '
                'ros2 run ros_gz_sim create'
                ' -name agro_robot'
                ' -string "$URDF"'
                ' -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z"'  # base_footprint on ground; base_link raised by wheel_radius via base_footprint_joint
            ),
            'spawn_robot',
            LaunchConfiguration('use_camera'),
            LaunchConfiguration('camera_width'),
            LaunchConfiguration('camera_height'),
            LaunchConfiguration('camera_rate'),
        ],
        name="spawn_robot",
        output="screen",
    )

    # ── 4. Bridge (2 s after a successful spawn) ─────────────────────────────
    # on_exit must be a callable, not a static list: a static list starts the
    # bridge whatever the spawn status, which publishes /clock for a world with
    # no robot and releases the Nav2/fusioncore clock gate.
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")

    def _bridge_if_spawn_ok(event, context):
        """Return the delayed bridge on spawn success, or a failure log otherwise."""
        if event.returncode != 0:
            return [LogInfo(msg=(
                f"[sim] spawn_robot exited with code {event.returncode}: "
                "not starting ros_gz_bridge"
            ))]
        return [
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        name="ros_gz_bridge",
                        output="screen",
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[
                            {"use_sim_time": True},
                            {"config_file": bridge_config},
                        ],
                    )
                ],
            )
        ]

    ros_gz_bridge = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=_bridge_if_spawn_ok)
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg,
        world_arg, urdf_arg, headless_arg,
        use_camera_arg, camera_width_arg, camera_height_arg, camera_rate_arg,
        gz_sim,
        gz_sim_headless,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])
