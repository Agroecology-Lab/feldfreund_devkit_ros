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
3. nav2 (t+15s)     — Nav2 server nodes with use_sim_time=True; by this point
                      /clock is live from the bridge and TF frames carry
                      Gazebo sim-time stamps, so all TF lookups succeed
4. nav2_lifecycle (t+20s) — lifecycle_manager_navigation, started 5s AFTER
                      the Nav2 nodes above so it never races their process
                      startup (previously both started in the same
                      TimerAction batch; under load this could leave
                      bt_navigator configured but never activated)
5. bridge_watchdog  — revives parameter_bridge if it dies (e.g. gz restart)

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
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            'x':     LaunchConfiguration('x'),
            'y':     LaunchConfiguration('y'),
            'z':     LaunchConfiguration('z'),
        }.items(),
    )

    bridge_cfg = os.path.join(pkg_agro, 'config', 'ros_gz_bridge.yaml')

    # Kill any stale bridge from a previous session before sim_launch starts.
    preflight_pkill = ExecuteProcess(
        cmd=['/bin/bash', '-c', 'pkill -f "gz sim" || true; pkill -f ros_gz_sim || true; pkill -f parameter_bridge || true'],
        name='preflight_pkill',
        output='screen',
    )

    # Bridge watchdog: revive parameter_bridge if it dies (e.g. gz restart).
    bridge_watchdog = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            (
                'while true; do sleep 5; '
                'pgrep -f parameter_bridge >/dev/null || '
                'ros2 run ros_gz_bridge parameter_bridge --ros-args '
                '-p config_file:=' + bridge_cfg + '; '
                'done'
            ),
        ],
        name='bridge_watchdog',
        output='screen',
    )

    # ── Nav2 (t+15s) ──────────────────────────────────────────────────────────
    # 15s gives gz sim time to start, the robot to spawn, and ros_gz_bridge
    # to come up and start publishing /clock with RELIABLE QoS.  By the time
    # Nav2 initialises, /clock is live and all TF frames carry Gazebo
    # sim-time stamps — so use_sim_time=True works correctly from the start.
    nav2_params = os.path.join(devkit_launch_pkg, 'config', 'nav2_params_sim.yaml')
    nav2 = TimerAction(
        period=15.0,
        actions=_nav2_sim_nodes(nav2_params, use_sim_time=True),
    )

    # ── Nav2 lifecycle manager (t+20s) ────────────────────────────────────────
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
        period=20.0,
        actions=[_nav2_lifecycle_manager_node(nav2_params, use_sim_time=True)],
    )

    # Kill the wall-time bootstrap TF publishers from sim_nav.launch.py once
    # Nav2 is up.  These were needed for topo nav localisation before Gazebo
    # started, but after the bridge is live their wall-time stamps look ancient
    # to Nav2's sim-time costmap (transform_tolerance=0.3s), causing
    # "Costmap timed out waiting for update" and zero cmd_vel output.
    # odom->base_footprint is now covered by the DiffDrive bridge on /tf;
    # base_footprint->base_link is covered by robot_state_publisher (50 Hz).
    # map->odom is left alone — it has no dynamic replacement.
    kill_bootstrap_tfs = TimerAction(
        period=16.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    '/bin/bash', '-c',
                    'ros2 lifecycle set /odom_to_base_footprint_static shutdown 2>/dev/null; '
                    'pkill -f "static_transform_publisher.*odom base_footprint" || true; '
                    'pkill -f "static_transform_publisher.*base_footprint base_link" || true; '
                    'echo "[bootstrap_tf_killer] killed odom->base_footprint and base_footprint->base_link static publishers"',
                ],
                name='kill_bootstrap_tfs',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        gz_resource_path,
        set_urdf_env,
        world_arg,
        urdf_arg,
        x_arg,
        y_arg,
        z_arg,
        preflight_pkill,
        sim_launch,
        bridge_watchdog,
        nav2,
        nav2_lifecycle,
        kill_bootstrap_tfs,
    ])
