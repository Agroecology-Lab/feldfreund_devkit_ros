"""
sowbot.launch.py
================
Simulation launch for the Sowbot agricultural robot.

Combines:
  - agro_robot_sim sim layer (Gazebo Harmonic + robot spawn + ros_gz_bridge)
  - devkit topo nav stack (map_manager2, localisation, navigation2, visualiser)
  - Real Nav2 backend (no fake_nav2_server, no AMCL, no map_server, no SLAM)

Topic wiring vs real hardware:
  - Gazebo publishes /odom (odom→base_footprint TF + nav_msgs/Odometry)
  - We relay /odom → /odom/wheels so fusioncore-style consumers are happy
  - Nav2 odom_topic set to /odom (not /fusion/odom — no fusioncore in sim)
  - /cmd_vel bridged Gazebo↔ROS by ros_gz_bridge in sim.launch.py
  - velocity_smoother output remapped cmd_vel_smoothed → cmd_vel so it
    actually reaches the bridge
  - /clock bridged → use_sim_time: true on all nodes, including the topo
    nav stack (otherwise TF lookups against sim-time stamps fail)

Robot model:
  sim.launch.py spawns urdf:=sowbot_01.xacro (Amiga-NG primitive geometry).
  To revert to the original agro_robot model omit the urdf argument or pass
  urdf:=agro_robot.urdf.xacro.

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
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Nav2 nodes — real Nav2 against sim time, no collision_monitor
# ---------------------------------------------------------------------------

def _nav2_sim_nodes(params_file: str) -> list:
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {
        "output": 'screen',
        "parameters": [{'use_sim_time': True}, params_file],
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
            parameters=[{'use_sim_time': True}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(package='nav2_smoother',  executable='smoother_server',   name='smoother_server',   **common),
        Node(package='nav2_planner',   executable='planner_server',    name='planner_server',    **common),
        Node(package='nav2_route',     executable='route_server',      name='route_server',      **common),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            parameters=[{'use_sim_time': True}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(package='nav2_bt_navigator',      executable='bt_navigator',      name='bt_navigator',      **common),
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
            parameters=[{'use_sim_time': True}, params_file],
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
                {'use_sim_time': True},
                {'autostart': True},
                params_file,
            ],
        ),
    ]


# ---------------------------------------------------------------------------
# Topo nav nodes
# ---------------------------------------------------------------------------

def _topo_nav_nodes(tmap2_file: str, devkit_launch_pkg: str) -> list:
    topo_share = get_package_share_directory('topological_navigation')
    map_path = tmap2_file or os.path.join(
        topo_share, 'config', 'mixed_actions_map.yaml')

    nav2_params = os.path.join(devkit_launch_pkg, 'config', 'nav2_params_sim.yaml')

    sim_time = {'use_sim_time': True}

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

        TimerAction(period=8.0, actions=_nav2_sim_nodes(nav2_params)),

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
# generate_launch_description
# ---------------------------------------------------------------------------

def generate_launch_description():
    pkg_agro          = get_package_share_directory('agro_robot_sim')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')

    tmap2_file = os.getenv('TMAP2_FILE', '')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='minha_fazenda.sdf',
        description='SDF world file name inside agro_robot_sim/worlds/',
    )

    # ── Gazebo sim layer ──────────────────────────────────────────────────────
    # Spawns sowbot_01.xacro (Amiga-NG visual model).
    # sim.launch.py handles: gz sim, robot_state_publisher, spawn_entity,
    # ros_gz_bridge for /cmd_vel /odom /tf /scan /imu /gps/fix /clock.
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agro, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'urdf':  'sowbot_01.xacro',
        }.items(),
    )

    # ── /odom → /odom/wheels relay ────────────────────────────────────────────
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_wheels_relay',
        arguments=['/odom', '/odom/wheels'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Static map → odom TF ──────────────────────────────────────────────────
    # Sole publisher of map→odom in sim (no AMCL, no fusioncore, no fake nav2).
    # TODO: replace with GPS-anchored origin once tmap2 nodes are surveyed.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Topo nav + Nav2 (delayed until Gazebo + robot spawn settle) ───────────
    topo_stack = TimerAction(
        period=5.0,
        actions=_topo_nav_nodes(tmap2_file, devkit_launch_pkg),
    )

    return LaunchDescription([
        world_arg,
        sim_launch,
        odom_relay,
        map_to_odom,
        topo_stack,
    ])
