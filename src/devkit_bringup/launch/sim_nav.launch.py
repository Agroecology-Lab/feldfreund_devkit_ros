"""
sim_nav.launch.py
=================
Nav-only launch for sim mode — started by manage.py on container boot.

Starts: topo nav stack (map_manager2, localisation2, navigation2,
        topological_map_visualiser) + UI node + bootstrap TFs.

Does NOT start: Gazebo, Nav2, fake_nav2_server.

Nav2 now starts inside sowbot_sim.launch.py (the Gazebo layer), so it
only ever initialises after /clock is already being published by the
ros_gz_bridge.  This eliminates the entire class of
"Extrapolation Error: Requested time X but earliest data is at Y"
failures that occurred when Nav2 started with use_sim_time=True before
Gazebo was up.

fake_nav2_server is removed.  It was providing:
  1. Continuous TF stream for localisation2  — replaced by the static
     bootstrap TFs below, which is sufficient (localisation2 seeds from
     the static chain and then tracks the real robot_state_publisher TF
     once Gazebo starts).
  2. /navigate_to_pose before Gazebo  — not needed; topo nav waits for
     localisation before accepting goals, and Nav2 now starts with Gazebo.
  3. /clock  — not needed; topo nav runs with use_sim_time=False so it
     does not need a /clock source.

fusioncore is now live in sim (previously ground-truth /odom was just
relayed straight to /odometry/global as a stand-in — see git history).
The ghost DiffDrive plugin in sowbot_01.xacro no longer publishes TF;
fusioncore is the sole publisher of odom->base_link, fed by /gnss/fix
and /imu/data (bridged) and /odom/wheels (relayed from ground-truth
/odom below, for encoder input only — this relay does NOT touch TF
anymore). Brought up via lifecycle configure/activate through the
launch event bus rather than `ros2 lifecycle set`, same pattern as
github.com/manankharwar/fusioncore/tree/main/fusioncore_gazebo.

TF tree at boot (before Gazebo starts)
---------------------------------------
  map --[static, wall-time]--> odom --[static]--> base_footprint
                                                        |
                                                   [static]
                                                        |
                                                   base_link

Once Gazebo + fusioncore start, fusioncore publishes the real dynamic
odom->base_link chain; TF2 uses the most recent transform so the
dynamic one supersedes the static bootstraps automatically.

NOTE: the base_footprint->base_link static bootstrap below and
fusioncore's dynamic odom->base_link publish both end at base_link
with different parents (base_footprint vs odom) — pre-existing from
when DiffDrive owned this edge, not introduced by the fusioncore
change. Only matters during the boot window before fusioncore is
active; TF2 tolerates the redundant edge but it's worth collapsing
properly at some point.

Startup sequencing
------------------
t+0s   map_manager2 starts, publishes topo map
t+2s   localisation2 starts, waits for map, then listens for TF
t+4s   navigation2 starts, waits for localisation
t+4s   fusioncore configure -> activate (lifecycle)
t+5s   topological_map_visualiser starts
UI     user presses Start Sim -> sowbot_sim.launch.py starts Gazebo +
       ros_gz_bridge + Nav2 (all with use_sim_time=True, /clock already live)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    topo_share = get_package_share_directory('topological_navigation')

    tmap2_file = os.getenv('TMAP2_FILE', '')
    map_path   = tmap2_file or os.path.join(
        topo_share, 'config', 'mixed_actions_map.yaml'
    )

    # use_sim_time=False throughout: no Gazebo, no /clock at this stage.
    # Topo nav only needs wall-time TF to localise; it does not plan paths.
    sim_time = {'use_sim_time': False}

    # ── /odom -> /odom/wheels relay ───────────────────────────────────────────
    # Bridges /odom onto /odom/wheels for edge-action consumers that expect
    # the wheel-odometry topic name.
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_wheels_relay',
        arguments=['/odom', '/odom/wheels'],
        parameters=[sim_time],
        output='screen',
    )

    # ── fusioncore (UKF localisation) ──────────────────────────────────────────
    # Sole publisher of odom->base_link in sim (ghost DiffDrive plugin in
    # sowbot_01.xacro no longer has <tf_topic>). Publishes /fusion/odom;
    # limbic_row_follow_node subscribes to /odometry/global, so a plain
    # relay maps one onto the other (same nav_msgs/Odometry type both ends).
    # Fed by /gnss/fix, /imu/data (bridged from Gazebo sensors) and
    # /odom/wheels (relay of ground-truth /odom below, encoder input only —
    # this does NOT feed TF, fusioncore owns that).
    fusioncore_params = PathJoinSubstitution(
        [FindPackageShare('devkit_bringup'), 'config', 'fusioncore_sim.yaml']
    )
    fusioncore_node = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        namespace='',
        output='screen',
        parameters=[fusioncore_params, sim_time],
    )

    # Configure -> activate through the launch event bus rather than
    # `ros2 lifecycle set`, same pattern as
    # github.com/manankharwar/fusioncore/tree/main/fusioncore_gazebo
    # (avoids node-discovery flakiness with an external CLI call).
    fusioncore_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(fusioncore_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    fusioncore_activate_on_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=fusioncore_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(fusioncore_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    fusioncore_bringup = TimerAction(period=4.0, actions=[
        fusioncore_node,
        fusioncore_activate_on_configured,
        fusioncore_configure,
    ])

    fusion_to_global_relay = TimerAction(period=4.0, actions=[
        Node(
            package='topic_tools',
            executable='relay',
            name='fusion_odom_global_relay',
            arguments=['/fusion/odom', '/odometry/global'],
            parameters=[sim_time],
            output='screen',
        ),
    ])

    # ── Bootstrap odom -> base_footprint (static, wall-time) ──────────────────
    # robot_state_publisher (Gazebo layer) normally provides this.  Without
    # it, localisation2's TF listener can never resolve map->base_link and
    # blocks forever.  Once Gazebo starts, the dynamic TF supersedes this
    # static one automatically.
    odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[sim_time],
        output='screen',
    )

    # ── Bootstrap base_footprint -> base_link (static, wall-time) ────────────
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[sim_time],
        output='screen',
    )

    # ── UI node ───────────────────────────────────────────────────────────────
    ui_node = Node(
        package='devkit_ui',
        executable='ui_node',
        name='ui_node',
        output='screen',
        respawn=True,
        respawn_delay=5,
        parameters=[{'sim': True}],
    )

    # ── Bootstrap map -> odom (static, wall-time) ────────────────────────────
    # map_manager2's broadcast_tf publishes map->map (a useless self-transform).
    # We publish map->odom ourselves so localisation2 can resolve map->base_link
    # via the full static chain: map->odom->base_footprint->base_link.
    # Once Gazebo starts, the bridge publishes real odom TF with sim-time stamps
    # which supersede this static transform automatically.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[sim_time],
        output='screen',
    )

    # ── map_manager2 ──────────────────────────────────────────────────────────
    # broadcast_tf=False: suppresses map_manager2's spurious map->map
    # self-transform; we publish map->odom ourselves above.
    map_manager = Node(
        package='topological_navigation',
        executable='map_manager2.py',
        name='topological_map_manager_2',
        output='screen',
        arguments=[map_path],
        parameters=[sim_time, {'broadcast_tf': False}],
    )

    # ── localisation2 (t+2s) ──────────────────────────────────────────────────
    # Needs the topo map (ready almost immediately) and TF map->base_link
    # (provided by the static bootstraps above).
    localisation = TimerAction(period=2.0, actions=[
        Node(
            package='topological_navigation',
            executable='localisation2.py',
            name='topological_localisation',
            output='screen',
            parameters=[sim_time],
        ),
    ])

    # ── navigation2 (t+4s) ────────────────────────────────────────────────────
    # Waits for localisation to signal ready before accepting goals.
    navigation = TimerAction(period=4.0, actions=[
        Node(
            package='topological_navigation',
            executable='navigation2.py',
            name='topological_navigation',
            output='screen',
            parameters=[sim_time],
        ),
    ])

    # ── limbic_row_follow action server (t+4s) ────────────────────────────────
    # Provides /limbic_row_follow so topo nav can hand off to the vision
    # pipeline on limbic_row_follow edges. Gated in devkit.launch.py to
    # real hardware only (comment said Neo's /row_follow/enable wouldn't
    # exist in sim — it does when neo.launch.py is running). Start at the
    # same time as navigation2 so the action server is ready before any
    # goal arrives. Subscribes to /aoc/heartbeat/neo_vision and calls
    # /row_follow/enable, both published by crop_row_node (neo.launch.py).
    row_follow_params = PathJoinSubstitution(
        [FindPackageShare('sowbot_row_follow'), 'config', 'crop_row_params.yaml']
    )
    limbic_row_follow = TimerAction(period=4.0, actions=[
        Node(
            package='sowbot_row_follow',
            executable='limbic_row_follow',
            name='limbic_row_follow',
            output='screen',
            parameters=[
                row_follow_params,
            ],
        ),
    ])

    # ── topological_map_visualiser (t+5s) ─────────────────────────────────────
    visualiser = TimerAction(period=5.0, actions=[
        Node(
            package='topological_navigation_visual',
            executable='topological_map_visualiser.py',
            name='topological_map_visualiser',
            output='screen',
            parameters=[sim_time, {'edit_mode': True}],
        ),
    ])

    return LaunchDescription([
        odom_relay,
        fusioncore_bringup,
        fusion_to_global_relay,
        map_to_odom,
        odom_to_base_footprint,
        base_footprint_to_base_link,
        ui_node,
        map_manager,
        localisation,
        navigation,
        limbic_row_follow,
        visualiser,
    ])
