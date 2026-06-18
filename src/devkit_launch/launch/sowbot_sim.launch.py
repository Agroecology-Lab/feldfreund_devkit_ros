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
3. nav2 (t+15s)     — Nav2 nodes with use_sim_time=True; by this point
                      /clock is live from the bridge and TF frames carry
                      Gazebo sim-time stamps, so all TF lookups succeed
4. bridge_watchdog  — revives parameter_bridge if it dies (e.g. gz restart)

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
        Node(package='nav2_route',             executable='route_server',      name='route_server',      **common),
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
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                params_file,
            ],
        ),
    ]


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
    pkg_agro          = get_package_share_directory('agro_robot_sim')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maize.world',
        description='SDF world file name inside agro_robot_sim/worlds/',
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

    # ── Gazebo + robot_state_publisher + spawn + ros_gz_bridge ───────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agro, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'urdf':  'sowbot_01.xacro',
            'x':     LaunchConfiguration('x'),
            'y':     LaunchConfiguration('y'),
            'z':     LaunchConfiguration('z'),
        }.items(),
    )

    bridge_cfg = os.path.join(pkg_agro, 'config', 'ros_gz_bridge.yaml')

    # Kill any stale bridge from a previous session before sim_launch starts.
    preflight_pkill = ExecuteProcess(
        cmd=['/bin/bash', '-c', 'pkill -f parameter_bridge || true'],
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

    return LaunchDescription([
        gz_resource_path,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        preflight_pkill,
        sim_launch,
        bridge_watchdog,
        nav2,
    ])
