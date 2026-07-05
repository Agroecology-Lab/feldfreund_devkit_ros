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

fusioncore now lives in sowbot_sim.launch.py (the Gazebo layer), NOT here,
for the same reason Nav2 does: it consumes sim-time-stamped sensors and must
run with use_sim_time=True after /clock is live. The ghost DiffDrive plugin in
sowbot_01.xacro no longer publishes TF; fusioncore is the sole publisher of
odom->base_footprint, fed by /gnss/fix and /imu/data (bridged) and /odom/wheels
(relayed from ground-truth /odom below, for encoder input only — this relay does
NOT touch TF). This file keeps only the wall-time bootstrap TFs and topo nav.

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
t+5s   topological_map_visualiser starts
UI     user presses Start Sim -> sowbot_sim.launch.py starts Gazebo +
       ros_gz_bridge + Nav2 + fusioncore (all with use_sim_time=True,
       /clock already live)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _read_spawn_xy(path='/workspace/spawn_pose.txt'):
    """Return (x, y) the robot will spawn at, in map/topo frame.

    map->odom was hardcoded to identity, which is only correct if the
    robot spawns exactly at map-frame (0,0). It never does — spawn_pose.txt
    holds the topo node the robot actually spawns at (e.g. (-1.106,
    -6.087)), written by topo_to_forest3d.py before this launch file runs
    in the same manage.py invocation. fusioncore's odom frame originates
    at wherever the robot is when its first GNSS fix lands (i.e. the
    spawn point), so with map->odom=identity, Nav2 reads the robot's
    odom-frame position (near (0,0)) as if it were the map-frame position
    — off by the entire spawn offset. Every goal, and the costmap, is
    then wrong by that same vector: the robot confidently drives toward
    real topo coordinates while believing it's somewhere else entirely,
    which is how it ends up off the terrain mesh.

    Spawn yaw is always 0 (sim.launch.py's spawn_robot never passes a yaw
    to `ros2 run ros_gz_sim create`), so only x/y need correcting here.
    """
    try:
        with open(path) as f:
            x, y, _z = f.read().split()
        return float(x), float(y)
    except Exception as e:
        print(f"[sim_nav] WARNING: could not read spawn pose from {path} "
              f"({e}) -- map->odom will fall back to identity, which is "
              "only correct if the robot spawns at map-frame (0,0).")
        return 0.0, 0.0


def generate_launch_description():
    topo_share = get_package_share_directory('topological_navigation')

    tmap2_file = os.getenv('TMAP2_FILE', '')
    map_path   = tmap2_file or os.path.join(
        topo_share, 'config', 'mixed_actions_map.yaml'
    )

    spawn_x, spawn_y = _read_spawn_xy()

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
    # MOVED to sowbot_sim.launch.py (the Gazebo layer), for the same reason Nav2
    # lives there: fusioncore consumes sim-time-stamped sensors (/gnss/fix,
    # /imu/data, /odom/wheels) and must run with use_sim_time=True *after* /clock
    # is live. Started here on wall time it stamped its odom->base_footprint TF
    # and /fusion/odom with wall-clock time while the rest of the stack is on sim
    # time, so everything it published looked ~1.7e9 s in the future -> the
    # "Extrapolation Error" class of failures and the robot driving off the world.
    # The /odom -> /odom/wheels relay above stays here: it is a stamp-preserving
    # passthrough and is harmless on wall time.

    # ── Bootstrap odom -> base_footprint (static, wall-time) ──────────────────
    # robot_state_publisher (Gazebo layer) normally provides this.  Without
    # it, localisation2's TF listener can never resolve map->base_link and
    # blocks forever.  Once Gazebo starts, the dynamic TF supersedes this
    # static one automatically.
    # NOTE: new-style named args (--x/--frame-id/etc), NOT the old positional
    # 'x y z yaw pitch roll frame child' form. static_transform_publisher's
    # old-style arg parser publishes onto plain /tf instead of the latched
    # /tf_static. That meant this bootstrap static and fusioncore's real,
    # moving odom->base_footprint TF were BOTH live on /tf for the entire
    # ~3min window before kill_bootstrap_tfs (sowbot_sim.launch.py) fires.
    # TF2 resolves lookups by most-recent-timestamp, so the controller
    # aliased between the frozen identity pose and fusioncore's real one —
    # a stable, periodic corruption that shows up as a perfect constant-
    # curvature circle in /odom. Publishing to /tf_static means
    # kill_bootstrap_tfs only has to clear a single latched transform, not
    # race a live publisher.
    odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_static',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'odom', '--child-frame-id', 'base_footprint',
        ],
        parameters=[sim_time],
        output='screen',
    )

    # ── Bootstrap base_footprint -> base_link (static, wall-time) ────────────
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_static',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
        ],
        parameters=[sim_time],
        output='screen',
    )

    # ── UI node ───────────────────────────────────────────────────────────────
    # Deliberate exception to this file's wall-time convention (see sim_time
    # above). Everything else here runs before Gazebo/clock exist, but
    # ui_node's _robot_pose() reads the live map->base_link TF for the SVG
    # marker, and that TF's timestamp comes from fusioncore — which now runs
    # in sowbot_sim.launch.py on use_sim_time=True (see that file). Without
    # this, ui_node's own get_clock().now() stays on wall time while the TF
    # stamp is in sim time; the staleness check in _robot_pose() then sees a
    # ~1.7e9s gap every time and blanks the marker unconditionally, even
    # though the transform itself is perfectly fresh. Harmless before Gazebo
    # starts: with no /clock yet, a sim-time node's clock just reads 0, which
    # still passes the staleness comparison against the (also wall-time)
    # bootstrap TFs below.
    ui_node = Node(
        package='devkit_ui',
        executable='ui_node',
        name='ui_node',
        output='screen',
        respawn=True,
        respawn_delay=5,
        parameters=[{'sim': True}, {'use_sim_time': True}],
    )

    # ── Bootstrap map -> odom (static, wall-time) ────────────────────────────
    # map_manager2's broadcast_tf publishes map->map (a useless self-transform).
    # We publish map->odom ourselves so localisation2 can resolve map->base_link
    # via the full static chain: map->odom->base_footprint->base_link.
    # Once Gazebo starts, the bridge publishes real odom TF with sim-time stamps
    # which supersede this static transform automatically.
    # Same old-style -> new-style fix as the two statics above, for
    # consistency (map->odom has no dynamic publisher to conflict with, but
    # it should still be latched on /tf_static rather than live on /tf so
    # late-starting TF listeners don't have to catch it mid-stream).
    #
    # Translation is (spawn_x, spawn_y), NOT (0,0,0) -- see _read_spawn_xy()
    # docstring above. This was the actual bug: identity here silently
    # offset the robot's believed map-frame position by the entire spawn
    # vector for the life of the run, since nothing else ever corrects
    # map->odom.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=[
            '--x', str(spawn_x), '--y', str(spawn_y), '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
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
