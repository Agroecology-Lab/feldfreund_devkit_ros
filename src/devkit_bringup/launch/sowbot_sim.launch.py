"""
sowbot_sim.launch.py
====================
Gazebo sim layer for the Sowbot — launched from the UI System tab.

Starts: Gazebo (gz sim), robot_state_publisher, spawn agro_robot,
        ros_gz_bridge, Nav2.

Does NOT start: topo nav, UI node.  Those run in sim_nav.launch.py
which manage.py starts at container boot.

Nav2 lives here (not in sim_nav.launch.py) so that it only ever
initialises after /clock is already being published by ros_gz_bridge.
This guarantees use_sim_time=True works correctly from the start — no
fake clock source, no race condition.

Startup sequence inside this file
----------------------------------
1. preflight_pkill  — kills any stale parameter_bridge from a previous run
2. sim_launch       — gz sim + robot_state_publisher + spawn + ros_gz_bridge
                      (ros_gz_bridge starts 2s after spawn exits, per sim.launch.py)
3. nav2 (t+35s)     — Nav2 server nodes with use_sim_time=True; by this point
                      /clock is live from the bridge and TF frames carry
                      Gazebo sim-time stamps, so all TF lookups succeed
4. nav2_lifecycle (t+40s) — lifecycle_manager_navigation, started 5s AFTER
                      the Nav2 nodes above so it never races their process
                      startup (previously both started in the same
                      TimerAction batch; under load this could leave
                      bt_navigator configured but never activated)

Nav2 node helpers (_nav2_sim_nodes, _topo_nav_nodes) are kept here so
that any future callers can import them via importlib if needed.

Robot model:
  Spawns urdf:=sowbot_01.xacro (Amiga-NG primitive geometry) by default.

Topo map:
  Set TMAP2_FILE env var to override. Defaults to mixed_actions_map.yaml
  from the topological_navigation share directory (the upstream demo map).

collision_monitor: omitted — Nav2 Jazzy crashes before lifecycle if no
  sensor is declared. Re-enable once confirmed working.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

# ---------------------------------------------------------------------------
# Nav2 nodes — always use_sim_time=True (Gazebo is already running by the
# time these are called)
# ---------------------------------------------------------------------------

def _nav2_sim_nodes(params_file: str, use_sim_time: bool = True) -> list:
    """Nav2 server/processing nodes only (excludes the lifecycle manager —
    see _nav2_lifecycle_manager_node() below, which is started on its own
    delayed timer so it never races the process startup of these nodes)."""
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {
        'output': 'screen',
        'parameters': [{'use_sim_time': use_sim_time}, params_file],
        'arguments': ['--ros-args', '--log-level', 'info'],
        'remappings': remappings,
    }

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            remappings=remappings,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(package='nav2_smoother',          executable='smoother_server',   name='smoother_server',   **common),
        Node(package='nav2_planner',           executable='planner_server',    name='planner_server',    **common),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=remappings,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(package='nav2_bt_navigator',      executable='bt_navigator',      name='bt_navigator',      **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', **common),
        # velocity_smoother intentionally omitted in sim: its output on /cmd_vel
        # races with joystick cmd_vel and publishes zeros when Nav2 is idle,
        # preventing manual driving.  controller_server and behavior_server
        # publish directly to /cmd_vel → ros_gz_bridge → Gazebo DiffDrive.
        # collision_monitor intentionally omitted — see module docstring.
        # docking_server intentionally omitted — no dock in sim world.
    ]


def _nav2_lifecycle_manager_node(params_file: str, use_sim_time: bool = True) -> Node:
    """lifecycle_manager_navigation, split out from _nav2_sim_nodes() and
    started on its own delayed timer (see `nav2_lifecycle` below).

    Root cause this addresses: previously the lifecycle manager launched in
    the SAME TimerAction batch as the nodes it manages, so all processes
    forked at the same instant. ros2 launch does not guarantee a node's
    rclpy context — let alone its lifecycle service servers — is up the
    moment the process forks. Under load (gz sim RTF has been observed
    running low on this stack) bt_navigator/behavior_server can take longer
    to come up than the lifecycle manager's default 4s bond_timeout, so the
    bond silently fails and bt_navigator is left configured but never
    activated — the exact "not auto-activated" symptom seen in testing.
    Splitting the manager onto a later timer plus a generous bond_timeout
    gives every managed node a real head start before lifecycle transitions
    are attempted."""
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'bond_timeout': 15.0},
            {'attempt_respawn_reconnection': True},
            params_file,
        ],
    )


# ---------------------------------------------------------------------------
# Topo nav nodes (kept for importlib callers; not used by this file directly)
# ---------------------------------------------------------------------------

def _topo_nav_nodes(tmap2_file: str, devkit_launch_pkg: str, use_sim_time: bool = True) -> list:
    """Topo nav node list — importlib shim for backwards compatibility."""
    topo_share = get_package_share_directory('topological_navigation')
    map_path = tmap2_file or os.path.join(topo_share, 'config', 'mixed_actions_map.yaml')

    nav2_params = os.path.join(devkit_launch_pkg, 'config', 'nav2_params_sim.yaml')
    sim_time = {'use_sim_time': use_sim_time}

    return [
        Node(
            package='topological_navigation',
            executable='map_manager2.py',
            name='topological_map_manager_2',
            output='screen',
            arguments=[map_path],
            parameters=[sim_time],
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package='topological_navigation',
                executable='localisation2.py',
                name='topological_localisation',
                output='screen',
                parameters=[sim_time],
            ),
        ]),

        TimerAction(period=8.0, actions=_nav2_sim_nodes(nav2_params, use_sim_time=use_sim_time)),
        TimerAction(period=12.0, actions=[
            _nav2_lifecycle_manager_node(nav2_params, use_sim_time=use_sim_time),
        ]),

        TimerAction(period=4.0, actions=[
            Node(
                package='topological_navigation',
                executable='navigation2.py',
                name='topological_navigation',
                output='screen',
                parameters=[sim_time],
            ),
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='topological_navigation_visual',
                executable='topological_map_visualiser.py',
                name='topological_map_visualiser',
                output='screen',
                parameters=[sim_time, {'edit_mode': True}],
            ),
        ]),
    ]


# ---------------------------------------------------------------------------
# generate_launch_description — Gazebo sim layer + Nav2
# ---------------------------------------------------------------------------

def generate_launch_description():
    """Build the simulation bringup with world generation, Gazebo, and navigation.

    Return a launch description that cleans up prior simulation processes and
    runs world generation. Gazebo, the /clock readiness gate and the map-to-odom
    update are scheduled only if world generation exits zero. Nav2, its lifecycle
    manager, fusioncore and the fusion odometry relay are scheduled only if the
    gate sees /clock within 240 seconds (5 s after for the nodes, 10 s for the
    lifecycle manager); otherwise none of them starts and the gate logs an error.
    Bootstrap base TF publishers are stopped only after a matching odom-to-base
    transform arrives on /tf; a timeout after 240 seconds or a failed wait leaves
    them running.

    Forward the camera launch arguments to the simulation: ``use_camera``
    defaults to true, ``camera_width`` and ``camera_height`` to 320 and 240
    pixels, and ``camera_rate`` to 10 Hz.

    Raise PackageNotFoundError if devkit_simulation or devkit_bringup cannot be
    found in the ament index.
    """
    pkg_agro          = get_package_share_directory('devkit_simulation')
    devkit_launch_pkg = get_package_share_directory('devkit_bringup')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maize.world',
        description='SDF world file name inside devkit_simulation/worlds/',
    )
    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value='sowbot_01.xacro',
        description='URDF/xacro filename inside devkit_simulation/urdf/. '
                    'Use sowbot_01.xacro (TrackedVehicle) or '
                    'robo_caatinga.urdf.xacro (DiffDrive skid-steer).',
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true: gz sim server-only, no GUI',
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
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.3')

    # Ensure Forest3D-generated models are always findable by gz sim.
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        '/workspace/models'
        ':/workspace/install/virtual_maize_field'
        '/share/virtual_maize_field/models'
        + (':' + existing if existing else ''),
    )

    # DEVKIT_URDF is read by sim.launch.py at generate_launch_description()
    # time to resolve xacro_file_eager — launch_arguments can't be used there
    # because substitutions aren't yet evaluated when os.path.join runs.
    set_urdf_env = SetEnvironmentVariable('DEVKIT_URDF', LaunchConfiguration('urdf'))

    # ── Gazebo + robot_state_publisher + spawn + ros_gz_bridge ───────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agro, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'urdf':  LaunchConfiguration('urdf'),
            'headless': LaunchConfiguration('headless'),
            'use_camera': LaunchConfiguration('use_camera'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_rate': LaunchConfiguration('camera_rate'),
            'x':     LaunchConfiguration('x'),
            'y':     LaunchConfiguration('y'),
            'z':     LaunchConfiguration('z'),
        }.items(),
    )

    # Kill any stale bridge from a previous session before sim_launch starts.
    # Also kills fake_nav2_server (the boot-time /clock+TF stub started by
    # sim_nav.launch.py) — real Nav2, started later in this file, takes over
    # both the navigation backend and the /clock source (via ros_gz_bridge),
    # so leaving fake_nav2_server alive would mean two NavigateToPose action
    # servers competing for the same action name. See nav2_only.launch.py's
    # module docstring for the full rationale (this mirrors that file's
    # kill_fake_nav2 step for the other UI entry point).
    # BUG FIXED HERE: pkill -f matches a process's FULL command line, not just
    # the target program name. The old version's `pkill -f "gz sim"` ran
    # inside `/bin/bash -c 'pkill -f "gz sim" || true; ...'` — and that bash
    # invocation's own command line contains the literal substring "gz sim"
    # (embedded in the script text passed to -c). So the very first pkill
    # call matched its own parent shell and SIGTERM'd it before the other
    # three pkill calls (ros_gz_sim, parameter_bridge, fake_nav2_server) ever
    # ran. This is why preflight_pkill has died with exit code -15 on every
    # single launch, every time — self-inflicted, not an external kill.
    # Net effect: cleanup of stale processes from a prior (possibly
    # SIGKILL'd) session never actually happened, which can leave a leftover
    # gz sim / parameter_bridge / fake_nav2_server around to race or conflict
    # with the new session (duplicate /clock-like publishers, stale DDS
    # participants, etc.).
    #
    # Fix: bracket one character of each pattern (e.g. "[g]z sim" instead of
    # "gz sim"). As a regex this still matches the literal string "gz sim" in
    # a TARGET process's command line, but it no longer matches pkill's own
    # invocation text (which contains "[g]z sim", not "gz sim").
    preflight_pkill = ExecuteProcess(
        cmd=['/bin/bash', '-c',
             'pkill -f "[g]z sim" || true; '
             'pkill -f "[r]os_gz_sim" || true; '
             'pkill -f "[p]arameter_bridge" || true; '
             'pkill -f "[f]ake_nav2_server" || true'],
        name='preflight_pkill',
        output='screen',
    )

    # sim_launch was previously a plain top-level action, started at the same
    # instant as preflight_pkill (ros2 launch runs top-level actions
    # concurrently, not in list order — the numbered docstring above was
    # aspirational, not enforced). preflight_pkill's `pkill -f "gz sim"` then
    # raced the freshly-started `gz sim -r ...` process from sim_launch and
    # could kill it seconds after it started — gz_sim exits, spawn_robot
    # (still in its "waiting for gz sim" loop) gets torn down with it, no
    # entity ever spawns, and every TF/costmap error downstream (Nav2 seeing
    # two unconnected trees, etc.) is fallout from that, not an independent
    # bug. Gate sim_launch on preflight_pkill's exit so the kill always
    # finishes before Gazebo starts.

    # ── World generation (input-keyed, cached) ───────────────────────────────
    # The Gazebo world is derived FROM the topo map by the Forest3D pipeline,
    # and Gazebo reads it only at launch. The placement inputs are read from
    # the "settings:" block of the last forest3d.yaml the UI wrote
    # (plant_spacing, weed_density, etc.), defaulting to the pipeline's
    # defaults when none exists yet, so Launch Sim replays the exact
    # generation the user configured instead of silently resetting values.
    # The world is cached: a hash of (resolved settings, topo map, crop/weed
    # model files, uploaded terrain, and the generator script itself) is
    # stored next to the world; if it still matches on the next press the
    # expensive 40-60s regeneration is skipped and gz starts instantly. The
    # key file lives beside the world in install/ (same container lifetime),
    # so a fresh `manage.py up --sim` always regenerates once. sim_launch is
    # gated on this step's exit so gz sim never races a half-written world.
    # The logic lives in /workspace/worldgen.sh (mounted by manage.py) so the
    # Launch Sim button and the UI Rebuild button share one implementation.
    world_gen = ExecuteProcess(
        cmd=['/bin/bash', '/workspace/worldgen.sh'],
        name='world_gen',
        output='screen',
    )

    start_sim_after_preflight = RegisterEventHandler(
        OnProcessExit(
            target_action=preflight_pkill,
            on_exit=[world_gen],
        )
    )

    def _launch_sim_if_worldgen_ok(event, context):
        """Return the simulation launch on world generation success, or log its failure."""
        if event.returncode != 0:
            return [LogInfo(msg=(
                f'[sowbot_sim] world_gen exited with code {event.returncode} — '
                'not starting sim_launch (world may be missing/stale)'
            ))]
        return [sim_launch]

    def _if_worldgen_ok(actions):
        """Create an exit handler that gates actions on successful world generation."""
        # Gate downstream actions on world_gen exiting 0. The failure is
        # already logged by _launch_sim_if_worldgen_ok.
        def _handler(event, context):
            """Return the gated actions on success, or an empty list on failure."""
            return list(actions) if event.returncode == 0 else []
        return _handler

    start_sim_after_worldgen = RegisterEventHandler(
        OnProcessExit(
            target_action=world_gen,
            on_exit=_launch_sim_if_worldgen_ok,
        )
    )

    # ── /clock readiness gate ────────────────────────────────────────────────
    # Nav2, its lifecycle manager, fusioncore and the fusion relay all need
    # /clock live (use_sim_time=True) and the robot spawned. They used to start
    # on fixed timers (35s/40s) after world_gen, which raced on a slow machine
    # (low RTF) and started the whole nav stack even when gz sim, the spawn or
    # ros_gz_bridge had failed. ros_gz_bridge is only launched after the spawn
    # step exits (see sim.launch.py), so /clock appearing means gz is up, the
    # spawn step has finished and the bridge is forwarding.
    #
    # Single `ros2 topic echo --once` with the full timeout: one DDS
    # participant, not many short-lived ones (see kill_bootstrap_tfs below).
    # Exit non-zero on timeout so nothing downstream is released.
    clock_gate = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'start_ts=$SECONDS; '
            'if timeout 240 ros2 topic echo /clock --once >/dev/null 2>&1; then '
            '  echo "[clock_gate] /clock live after $((SECONDS-start_ts))s"; '
            'else '
            '  echo "[clock_gate] ERROR: no /clock after $((SECONDS-start_ts))s '
            '(gz sim, spawn or ros_gz_bridge did not come up) — '
            'not starting Nav2/fusioncore"; exit 1; '
            'fi',
        ],
        name='clock_gate',
        output='screen',
    )
    start_clock_gate_after_worldgen = RegisterEventHandler(
        OnProcessExit(target_action=world_gen, on_exit=_if_worldgen_ok([clock_gate])),
    )

    # Offsets below are measured from /clock first being seen, not from
    # world_gen's exit. Same 5s spacing as before: server nodes (and
    # fusioncore) first, lifecycle manager 5s later.
    NAV_START_AFTER_CLOCK_S = 5.0
    NAV_LIFECYCLE_AFTER_CLOCK_S = 10.0

    # ── Nav2 (5s after /clock is live) ───────────────────────────────────────
    # By the time Nav2 initialises, /clock is live and all TF frames carry
    # Gazebo sim-time stamps — so use_sim_time=True works correctly from the
    # start. Anchored on clock_gate's exit (see above), not a fixed sleep that
    # needs retuning whenever spawn gets slower.
    nav2_params = os.path.join(devkit_launch_pkg, 'config', 'nav2_params_sim.yaml')
    nav2 = TimerAction(
        period=NAV_START_AFTER_CLOCK_S,
        actions=_nav2_sim_nodes(nav2_params, use_sim_time=True),
    )
    start_nav2_after_clock = RegisterEventHandler(
        OnProcessExit(target_action=clock_gate, on_exit=_if_worldgen_ok([nav2])),
    )

    # ── Nav2 lifecycle manager (10s after /clock is live) ─────────────────────
    # Started 5s AFTER the Nav2 server nodes above, not in the same batch.
    # Previously this raced controller_server/behavior_server/bt_navigator's
    # process startup, since ros2 launch forks everything in a TimerAction
    # batch at the same instant with no guarantee the managed nodes' lifecycle
    # services are actually up yet. Under load (gz sim RTF has run low on
    # this stack) that race meant bt_navigator could be left configured but
    # never activated, which is exactly the "[NAV2] Server unavailable" /
    # STATUS_ABORTED failure topo nav goals were hitting. The 5s head start
    # plus bond_timeout=15.0 (see _nav2_lifecycle_manager_node) gives every
    # managed node real margin before lifecycle transitions are attempted.
    nav2_lifecycle = TimerAction(
        period=NAV_LIFECYCLE_AFTER_CLOCK_S,
        actions=[_nav2_lifecycle_manager_node(nav2_params, use_sim_time=True)],
    )
    start_nav2_lifecycle_after_clock = RegisterEventHandler(
        OnProcessExit(target_action=clock_gate, on_exit=_if_worldgen_ok([nav2_lifecycle])),
    )

    # Kill the wall-time bootstrap TF publishers from sim_nav.launch.py once
    # a REAL odom->base_footprint source is actually live.  These were needed
    # for topo nav localisation before Gazebo started, but after the bridge
    # is live their wall-time stamps look ancient to Nav2's sim-time costmap
    # (transform_tolerance=0.3s), causing "Costmap timed out waiting for
    # update" and zero cmd_vel output.
    #
    # This was previously a fixed TimerAction(period=16.0), measured from
    # generate_launch_description() being called — i.e. from before
    # preflight_pkill even ran, NOT from when sim_launch (gz sim + spawn +
    # bridge) actually started. spawn_robot/parameter_bridge routinely don't
    # finish until t+35-40s (gz sim startup + 30s GUI-init wait in
    # spawn_robot + bridge creation), so the fixed timer killed the bootstrap
    # statics 20+ seconds before any real replacement existed. Result: a
    # dead gap with NO odom->base_footprint publisher at all -> "Could not
    # find a connection between 'odom' and 'base_footprint' ... two or more
    # unconnected trees" (exactly the error seen in controller_server logs).
    #
    # Fix: poll for real /odom data (proof the DiffDrive bridge is actually
    # forwarding gz Odometry, not just that the bridge node/topic exists)
    # before killing anything. [STALE since DiffDrive publish_tf=false: the
    # trigger is now fusioncore's own TF, see below.] base_footprint->base_link
    # is covered by robot_state_publisher (fixed base_footprint_joint in the
    # xacro). map->odom is left alone — it has no dynamic replacement.
    #
    # NOTE: this was previously a loop of `timeout 2 ros2 topic echo /odom
    # --once` retried every 2s for up to 240s. That spawns a brand-new DDS
    # participant on every single attempt, and gives each one only 2s to
    # stand up, complete discovery against the bridge's existing publisher,
    # match, and receive a sample -- a budget that's routinely too tight
    # under loopback multicast discovery with ParticipantIndex=auto
    # (successive short-lived participants can be slowed by TIME_WAIT churn
    # on the discovery ports). Result: the check reported "no /odom data
    # after 240s" and permanently killed the bootstrap statics even while
    # /odom was being published continuously and reliably to every other
    # node in the graph -- a false negative in the check, not an actual
    # data outage. Fixed by using ONE participant with the full timeout
    # budget instead of 120 short-lived ones.
    # Trigger: a DYNAMIC odom->base_footprint (or ->base_link) transform on
    # /tf. The bootstrap statics are on /tf_static (new-style args), so any
    # /tf message with parent 'odom' can only come from fusioncore, the sole
    # live source. Waiting on /odom instead (the previous trigger) proves
    # nothing about fusioncore: /odom is gz ground truth, fusioncore starts on
    # its own 35s timer and only publishes once it has initialised. Killing on
    # /odom data therefore left a gap with no odom->base_footprint source.
    # On timeout the statics are KEPT: killing them with no replacement is the
    # "two or more unconnected trees" failure described above.
    odom_tf_filter = (
        "any(t.header.frame_id=='odom' and "
        "t.child_frame_id in ('base_footprint','base_link') "
        "for t in m.transforms)"
    )
    kill_bootstrap_tfs = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'start_ts=$SECONDS; timeout_s=240; '
            f'if timeout "$timeout_s" ros2 topic echo /tf --once --filter "{odom_tf_filter}" >/dev/null 2>&1; then '
            '  echo "[bootstrap_tf_killer] dynamic odom TF (fusioncore) live after $((SECONDS-start_ts))s"; '
            '  pkill -f "static_transform_publishe[r].*__node:=odom_to_base_footprint_static" || true; '
            '  pkill -f "static_transform_publishe[r].*__node:=base_footprint_to_base_link_static" || true; '
            '  echo "[bootstrap_tf_killer] killed odom->base_footprint and base_footprint->base_link static publishers ($((SECONDS-start_ts))s elapsed)"; '
            'else '
            '  echo "[bootstrap_tf_killer] WARNING: no dynamic odom TF on /tf after $((SECONDS-start_ts))s — keeping bootstrap statics (fusioncore not publishing?)"; '
            'fi',
        ],
        name='kill_bootstrap_tfs',
        output='screen',
    )

    # ── Fix map->odom (after world-gen writes spawn_pose.txt) ─────────────────
    # sim_nav.launch.py publishes map->odom at container boot, BEFORE
    # spawn_pose.txt exists (worldgen only runs when the user presses Launch
    # Sim), so _read_spawn_xy() falls back to identity (0,0,0). With identity,
    # fusioncore's odom->base_footprint — whose odom frame is anchored at the
    # robot's spawn point, so it sits near (0,0) at spawn — gets read as the
    # robot's MAP-frame position. That is off by the entire spawn offset: the
    # robot believes it is at map (0,0) while actually being at the spawn node
    # (e.g. R1_IN at (0,-3.528)). Every goal, the costmap and the UI topo map
    # are then wrong by that vector.
    #
    # worldgen writes spawn_pose.txt, so by the time world_gen exits we know
    # the real spawn. Kill the stale boot-time static and republish map->odom
    # with the spawn offset. tf2's static buffer keeps the first transform it
    # saw for a frame pair, so the stale one MUST be killed before republishing
    # — a second concurrent publisher for map->odom is ignored.
    fix_map_to_odom = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'SPAWN_FILE=/workspace/spawn_pose.txt; '
            'if [ ! -f "$SPAWN_FILE" ]; then '
            '  echo "[map_to_odom_fixer] WARNING: $SPAWN_FILE missing — '
            'leaving boot-time identity map->odom (alignment will be wrong)"; '
            'else '
            '  read -r SPAWN_X SPAWN_Y SPAWN_Z < "$SPAWN_FILE"; '
            '  echo "[map_to_odom_fixer] spawn ($SPAWN_X, $SPAWN_Y) from $SPAWN_FILE"; '
            '  echo "[map_to_odom_fixer] killing stale boot-time map->odom static"; '
            '  pkill -f "static_transform_publishe[r].*__node:=map_to_odom_static" || true; '
            '  sleep 1; '
            '  exec ros2 run tf2_ros static_transform_publisher '
            '    --x "$SPAWN_X" --y "$SPAWN_Y" --z 0 '
            '    --qx 0 --qy 0 --qz 0 --qw 1 '
            '    --frame-id map --child-frame-id odom '
            '    --ros-args -r __node:=map_to_odom_fixed; '
            'fi'
        ],
        name='map_to_odom_fixer',
        output='screen',
    )
    start_map_to_odom_fixer_after_worldgen = RegisterEventHandler(
        OnProcessExit(target_action=world_gen, on_exit=_if_worldgen_ok([fix_map_to_odom])),
    )

    # ── fusioncore (UKF localisation, 5s after /clock is live) ────────────────
    # MOVED here from sim_nav.launch.py. fusioncore consumes sim-time-stamped
    # sensors (/gnss/fix, /imu/data bridged from Gazebo, /odom/wheels relayed
    # from ground-truth /odom) and is the sole publisher of odom->base_footprint.
    # It MUST run with use_sim_time=True and only after /clock is live, exactly
    # like Nav2 above. Started on wall time it stamped its TF and /fusion/odom
    # with wall-clock time while the sim-time stack rejected them as ~1.7e9 s in
    # the future ("Extrapolation Error"), which sent the robot off the world.
    # Started on clock_gate's exit, same as Nav2: gz sim + spawn + ros_gz_bridge
    # are up and /clock is publishing by then. autostart (fusioncore >= 0.3.1) activates the
    # node ~200ms after configure, so a lone CONFIGURE event is enough.
    fusioncore_params = PathJoinSubstitution(
        [FindPackageShare('devkit_bringup'), 'config', 'fusioncore_sim.yaml']
    )
    fusioncore_node = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        namespace='',
        output='screen',
        parameters=[fusioncore_params, {'use_sim_time': True}],
    )
    fusioncore_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(fusioncore_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    fusioncore_bringup = TimerAction(period=NAV_START_AFTER_CLOCK_S, actions=[
        fusioncore_node,
        fusioncore_configure,
    ])
    start_fusioncore_after_clock = RegisterEventHandler(
        OnProcessExit(target_action=clock_gate, on_exit=_if_worldgen_ok([fusioncore_bringup])),
    )

    # /fusion/odom -> /odometry/global (limbic_row_follow subscribes to the
    # latter; same nav_msgs/Odometry type both ends).
    fusion_to_global_relay = TimerAction(period=NAV_START_AFTER_CLOCK_S, actions=[
        Node(
            package='topic_tools',
            executable='relay',
            name='fusion_odom_global_relay',
            arguments=['/fusion/odom', '/odometry/global'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
    start_fusion_to_global_relay_after_clock = RegisterEventHandler(
        OnProcessExit(target_action=clock_gate, on_exit=_if_worldgen_ok([fusion_to_global_relay])),
    )

    return LaunchDescription([
        gz_resource_path,
        set_urdf_env,
        world_arg,
        headless_arg,
        urdf_arg,
        use_camera_arg,
        camera_width_arg,
        camera_height_arg,
        camera_rate_arg,
        x_arg,
        y_arg,
        z_arg,
        preflight_pkill,
        start_sim_after_preflight,
        start_sim_after_worldgen,
        start_clock_gate_after_worldgen,
        start_nav2_after_clock,
        start_nav2_lifecycle_after_clock,
        start_fusioncore_after_clock,
        start_fusion_to_global_relay_after_clock,
        start_map_to_odom_fixer_after_worldgen,
        kill_bootstrap_tfs,
    ])
