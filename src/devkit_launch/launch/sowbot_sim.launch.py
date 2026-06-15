"""
sowbot_sim.launch.py
====================
Gazebo sim layer for the Sowbot — launched from the UI System tab.

Starts: Gazebo (gz sim), robot_state_publisher, spawn agro_robot,
ros_gz_bridge (via YAML config — see agro_robot_sim/config/ros_gz_bridge.yaml).

Does NOT start: topo nav, Nav2, UI node.  Those run in sim_nav.launch.py
which manage.py starts at container boot.  Keeping the layers separate means
the user can restart Gazebo from the UI without tearing down the nav stack.

Nav2 node helpers (_nav2_sim_nodes, _topo_nav_nodes) are defined here so that
sim_nav.launch.py can import them via importlib without duplicating code.

Robot model:
  Spawns urdf:=sowbot_01.xacro (Amiga-NG primitive geometry) by default.

Topo map:
  Set TMAP2_FILE env var to override. Defaults to mixed_actions_map.yaml
  from the topological_navigation share directory (the upstream demo map).

collision_monitor: omitted — Nav2 Jazzy crashes before lifecycle if no sensor
  is declared. Re-enable once confirmed working by adding it to
  _nav2_sim_nodes(), nav2_params_sim.yaml node_names, and a
  collision_monitor params block in the YAML.
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
# Nav2 nodes — real Nav2 against sim time, no collision_monitor
# ---------------------------------------------------------------------------

def _nav2_sim_nodes(params_file: str, use_sim_time: bool = True) -> list:
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {
        "output": 'screen',
        "parameters": [{'use_sim_time': use_sim_time}, params_file],
        "arguments": ['--ros-args', '--log-level', 'info'],
        "remappings": remappings,
    }

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(package='nav2_smoother',          executable='smoother_server',   name='smoother_server',    **common),
        Node(package='nav2_planner',           executable='planner_server',    name='planner_server',     **common),
        Node(package='nav2_route',             executable='route_server',      name='route_server',       **common),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(package='nav2_bt_navigator',      executable='bt_navigator',      name='bt_navigator',       **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', **common),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            remappings=remappings + [
                ('cmd_vel',          'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
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
# Topo nav nodes
# ---------------------------------------------------------------------------

def _topo_nav_nodes(tmap2_file: str, devkit_launch_pkg: str, use_sim_time: bool = True) -> list:
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
            # 'broadcast_tf': False explicitly suppresses the 'odom -> map' 
            # TF loop conflict with fake_nav2_server.
            parameters=[sim_time, {'broadcast_tf': False}],
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
# generate_launch_description — Gazebo sim layer only
# ---------------------------------------------------------------------------

def generate_launch_description():
    pkg_agro = get_package_share_directory('agro_robot_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='minha_fazenda.sdf',
        description='SDF world file name inside agro_robot_sim/worlds/',
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.3')

    # Ensure Forest3D-generated models (model://ground, model://crop/plant)
    # are always findable by gz sim regardless of the calling environment.
    # The UI's _SIM_ENV sets this too, but setting it here covers direct
    # `ros2 launch` invocations and any future callers.
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        '/workspace/models'
        ':/workspace/install/virtual_maize_field'
        '/share/virtual_maize_field/models'
        + (':' + existing if existing else ''),
    )

    # sim.launch.py: gz sim + robot_state_publisher + spawn + ros_gz_bridge
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

    # Preflight: kill any lingering bridge from a previous session before
    # sim_launch starts a fresh one.
    preflight_pkill = ExecuteProcess(
        cmd=['/bin/bash', '-c', 'pkill -f parameter_bridge || true'],
        name='preflight_pkill',
        output='screen',
    )

    # Bridge watchdog: if parameter_bridge dies (e.g. GZ restart), revive it.
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

    # Kill fake_nav2_server once /clock is being published RELIABLE from the
    # bridge.  topic hz has a startup lag and doesn't expose QoS; topic info
    # --verbose confirms the bridge is fully up with the right reliability.
    kill_fake_nav2 = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'until ros2 topic info /clock --verbose 2>/dev/null | grep -q RELIABLE; '
            'do sleep 1; done; '
            'pkill -f fake_nav2_server || true'
        ],
        name='kill_fake_nav2_on_clock',
        output='screen',
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
        kill_fake_nav2,
    ])
