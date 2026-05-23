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

Topo map:
  Set TMAP2_FILE env var to override. Defaults to mixed_actions_map.yaml
  from the topological_navigation share directory (the upstream demo map).
  Neither default fits the minha_fazenda.sdf world geometry. World rows
  are at y≈{0, 3, 6} and x≈{2..14}; the devkit's mixed_test_map sits at
  y=±{4, 8}, x={5..33}. The robot spawns at (0, 0, 0.3) — HOME (0, 0)
  is the only reachable node and there is nowhere meaningful to drive to.
  TODO: author a sowbot_test_map.yaml whose node positions match
        minha_fazenda.sdf, or replace the SDF with one scaled to match
        an existing map. Until then, end-to-end nav goals will not work.

collision_monitor: omitted — Nav2 Jazzy crashes before lifecycle if no sensor
  is declared. The URDF has a lidar_link with /scan; re-enable once confirmed
  working by adding it to _nav2_sim_nodes(), nav2_params_sim.yaml node_names,
  and a collision_monitor params block in the YAML.
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
    """
    Explicit Nav2 node declarations for sim.

    Mirrors _nav2_nodes() in devkit.launch.py but:
      - use_sim_time: True throughout
      - odom_topic: /odom  (Gazebo diff-drive, not /fusion/odom) — set in
        nav2_params_sim.yaml, not here
      - robot_base_frame: base_footprint (matches URDF diff-drive plugin's
        child_frame_id) — also set in nav2_params_sim.yaml
      - velocity_smoother gets an extra ('cmd_vel_smoothed', 'cmd_vel')
        remap so its output reaches the Gazebo bridge. The upstream Nav2
        navigation_launch.py applies the same remap; we recreate it here
        because we don't include navigation_launch.py.
      - collision_monitor omitted (same rationale as hardware — no sensor
        declared in params means Jazzy throws InvalidParameterValueException
        at startup; the URDF lidar is wired, add back once confirmed)
      - docking_server omitted — no dock in sim world. Must stay out of
        lifecycle_manager_navigation.node_names in the YAML too, or the
        manager will block waiting for it.
    """
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = dict(
        output='screen',
        parameters=[{'use_sim_time': True}, params_file],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings,
    )

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
        Node(package='nav2_smoother',   executable='smoother_server',  name='smoother_server',  **common),
        Node(package='nav2_planner',    executable='planner_server',   name='planner_server',   **common),
        Node(package='nav2_route',      executable='route_server',     name='route_server',     **common),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            parameters=[{'use_sim_time': True}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(package='nav2_bt_navigator',       executable='bt_navigator',    name='bt_navigator',    **common),
        Node(package='nav2_waypoint_follower',  executable='waypoint_follower', name='waypoint_follower', **common),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            # Input  : cmd_vel        -> cmd_vel_nav      (from controller_server)
            # Output : cmd_vel_smoothed -> cmd_vel         (to Gazebo bridge)
            # Without the output remap the smoother publishes to /cmd_vel_smoothed,
            # which the ros_gz_bridge does not subscribe to, and the robot
            # never moves under Nav2 control.
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
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
# Topo nav nodes — identical pattern to devkit.launch.py _topo_nav_nodes()
# ---------------------------------------------------------------------------

def _topo_nav_nodes(tmap2_file: str, devkit_launch_pkg: str) -> list:
    topo_share = get_package_share_directory('topological_navigation')
    map_path = tmap2_file or os.path.join(
        topo_share, 'config', 'mixed_actions_map.yaml')

    nav2_params = os.path.join(devkit_launch_pkg, 'config', 'nav2_params_sim.yaml')

    # All topo nodes run on sim time. Without this, /clock is being published
    # by the Gazebo bridge but the topo nodes use wall-clock for their
    # subscriptions and TF lookups, producing 'extrapolation into the past'
    # errors as soon as sim time and wall time diverge.
    sim_time = {'use_sim_time': True}

    return [
        # 1. Map manager
        Node(
            package='topological_navigation',
            executable='map_manager2.py',
            name='topological_map_manager_2',
            output='screen',
            arguments=[map_path],
            parameters=[sim_time],
        ),

        # 2. Localisation (2 s delay — map must be published first)
        TimerAction(period=2.0, actions=[
            Node(
                package='topological_navigation',
                executable='localisation2.py',
                name='topological_localisation',
                output='screen',
                parameters=[sim_time],
            ),
        ]),

        # 3. Real Nav2 (delayed 3 s — Gazebo + robot spawn need to settle)
        TimerAction(period=8.0, actions=_nav2_sim_nodes(nav2_params)),

        # 4. Topo navigation server (4 s)
        TimerAction(period=4.0, actions=[
            Node(
                package='topological_navigation',
                executable='navigation2.py',
                name='topological_navigation',
                output='screen',
                parameters=[sim_time],
            ),
        ]),

        # 5. Map visualiser (5 s)
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
    pkg_agro         = get_package_share_directory('agro_robot_sim')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')

    tmap2_file = os.getenv('TMAP2_FILE', '')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='minha_fazenda.sdf',
        description='SDF world file name inside agro_robot_sim/worlds/',
    )

    # ── Gazebo sim layer (from agro_robot_sim) ────────────────────────────────
    # Handles: gz sim, robot_state_publisher, spawn_entity, ros_gz_bridge
    # Bridges: /cmd_vel, /odom, /tf, /scan, /imu, /gps/fix, /clock
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agro, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # ── /odom → /odom/wheels relay ────────────────────────────────────────────
    # Mirrors devkit.launch.py; keeps the topic name available for any
    # consumer that subscribes to /odom/wheels (the hardware convention).
    # Nav2 itself reads /odom directly in sim — see nav2_params_sim.yaml.
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_wheels_relay',
        arguments=['/odom', '/odom/wheels'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Static map → odom TF ──────────────────────────────────────────────────
    # In sim, the Gazebo diff-drive plugin owns odom → base_footprint.
    # Topo nav and Nav2 need map → odom to exist; this node is the SOLE
    # publisher of that transform in the sim configuration (no AMCL, no
    # fake_nav2_server, no fusioncore). Anything else that publishes
    # map → odom will fight this static publisher.
    # TODO: replace with GPS-anchored origin once tmap2 nodes are surveyed.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Topo nav + Nav2 (delayed until Gazebo is ready) ───────────────────────
    # sim.launch.py spawns the robot after 3 s; give it 5 s total before
    # topo map_manager starts. Nav2 delayed a further 3 s inside _topo_nav_nodes.
    # 5 s is a guess — if Gazebo cold-starts slowly (first-run shader compile
    # under ogre2, slow disk, low CPU), bump it.
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
