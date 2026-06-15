"""
sim_nav.launch.py
=================
Nav-only launch for sim mode — started by manage.py on container boot.

Provides the full navigation stack (topo nav + Nav2 + UI) but does NOT
start Gazebo or spawn the robot.  Gazebo is started separately by the
user from the UI System tab, which runs sowbot_sim.launch.py and brings
up the sim layer (gz sim, robot_state_publisher, spawn, ros_gz_bridge).

Keeping these separate means:
  - manage.py starts the nav stack once at boot, fast.
  - The user controls when Gazebo opens (and can restart it without
    tearing down the whole nav stack).
  - No duplicate Gazebo instances when the UI button is pressed.

TF tree at boot (before Gazebo starts)
---------------------------------------
  map ──(static)──► odom ──(static)──► base_footprint ──(static)──► base_link
                                           │
                                     (identity — bootstrap only)

robot_state_publisher in the Gazebo layer publishes the real
odom → base_link → base_footprint chain on /tf once Gazebo is up.
Those dynamic transforms supersede these static ones automatically.
Without the bootstraps:
  - Nav2 costmaps block on base_footprint → odom forever and the action
    server never becomes ready.
  - localisation2.py blocks on map → base_link (hardcoded upstream
    default) and the action server never becomes ready.

Startup sequencing (why 15 s timer)
------------------------------------
map_manager2  starts at ~t+0 s and publishes the topo map.
localisation2 starts at ~t+2 s and signals ready after receiving the map
              and confirming the TF chain (~t+5 s).
navigation2   must NOT start waiting for localisation until localisation
              has had time to reach ready state.  With the 5 s timer the
              two were racing; raising to 15 s gives localisation a
              comfortable head start so navigation2 finds it immediately.
"""

import importlib.util
import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _load_sowbot_sim():
    """Load sowbot_sim.launch.py from the devkit_launch install path.

    Both packages are now peers in the same colcon workspace, but the
    importlib pattern is retained so the Nav2 / topo-nav node lists stay
    in one place (sowbot_sim.launch.py) without duplicating them here.
    """
    pkg  = get_package_share_directory('devkit_launch')
    path = pathlib.Path(pkg) / 'launch' / 'sowbot_sim.launch.py'
    assert path.exists(), (
        f"sowbot_sim.launch.py not found at {path} — "
        "build devkit_launch first and source the workspace"
    )
    spec = importlib.util.spec_from_file_location('sowbot_sim', path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    devkit_launch_pkg = get_package_share_directory('devkit_launch')
    sowbot_sim        = _load_sowbot_sim()

    tmap2_file = os.getenv('TMAP2_FILE', '')

    # ── /odom → /odom/wheels relay ────────────────────────────────────────────
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_wheels_relay',
        arguments=['/odom', '/odom/wheels'],
        parameters=[{'use_sim_time': True}],
    # nav2_params_sim.yaml sets robot_base_frame: base_footprint everywhere.
    # That frame is normally published by robot_state_publisher, which lives
    # in the Gazebo layer (sowbot_sim.launch.py).  Without Gazebo running,
    # Nav2 costmaps spin forever waiting for the transform and the action
    # server never becomes ready.
    #
    # This static identity TF unblocks Nav2 at boot.  Once Gazebo starts,
    # robot_state_publisher publishes the real chain on /tf (dynamic);
    # TF2 uses the most recent transform so the dynamic one takes over
    # without any intervention here.
    odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── Bootstrap base_footprint → base_link ──────────────────────────────────
    # localisation2.py listens for map → base_link (hardcoded upstream
    # default).  robot_state_publisher provides this once Gazebo starts;
    # this identity bootstrap unblocks localisation before Gazebo is up,
    # same pattern as the odom → base_footprint static above.
    #
    # Once Gazebo starts, robot_state_publisher publishes the real dynamic
    # chain and TF2 supersedes this static one automatically.
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── UI node (sim=true: publishes fake GPS fix for topo map saving) ────────
    ui_node = Node(
        package='devkit_ui',
        executable='ui_node',
        name='ui_node',
        output='screen',
        respawn=True,
        respawn_delay=5,
        parameters=[{'sim': True}],
    )

    # ── fake_nav2_server (pre-Gazebo TF source) ──────────────────────────────
    # localisation2 uses a TF listener callback that fires on each new
    # transform.  The static bootstrap TFs above publish once at startup and
    # then go silent, so the callback never fires and /current_node is never
    # published.  fake_nav2_server's VirtualRobot publishes map→odom→base_link
    # at 30 Hz, giving localisation2 the live TF stream it needs.
    #
    # This node is started HERE (sim_nav.launch.py) and NOT inside
    # _topo_nav_nodes(), so it runs only before Gazebo is up.  Once the user
    # starts Gazebo from the UI, robot_state_publisher takes over the TF chain
    # and fake_nav2_server's action servers must not compete with real Nav2 —
    # the user kills this node (or it is superseded) at that point.
    fake_nav2 = TimerAction(
        period=2.0,
        actions=[Node(
            package='topological_nav_simulator',
            executable='fake_nav2_server',
            name='fake_nav2_server',
            output='screen',
            parameters=[{'use_sim_time': False}],  # no /clock in sim_nav; wall time keeps TF timestamps valid
        )],
    )

    # ── Kill fake_nav2_server once /clock arrives from the bridge ─────────────
    # fake_nav2_server publishes map→odom→base_link at wall time. Once Gazebo
    # starts and ros_gz_bridge publishes /clock, its wall-time TF stamps race
    # with the sim-time TF from RSP/bridge and corrupt the robot's map position.
    # We poll for /clock and SIGTERM fake_nav2_server as soon as it appears.
    kill_fake_nav2 = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'until ros2 topic hz /clock --window 1 2>/dev/null | grep -q "average rate"; '
            'do sleep 1; done; '
            'pkill -f fake_nav2_server || true'
        ],
        name='kill_fake_nav2_on_clock',
        output='screen',
    )

    # ── Topo nav + Nav2 (delayed to let static TFs and topo stack settle) ───────
    # 15 s gives map_manager2 and localisation2 time to fully initialise
    # before navigation2 starts waiting for the localisation ready signal.
    # With 5 s the two were racing: navigation2 timed out before
    # localisation2 had finished building the KD-tree and confirming the
    # TF chain, causing the "action server not ready" error.
    # If Gazebo starts later, Nav2/topo-nav wait for real TF data before
    # accepting goals — the bootstrap TFs keep them unblocked in the interim.
    # use_sim_time=True: once Gazebo starts, /clock drives all timestamps so
    # localisation2 and Nav2 stay in sync with the bridge's odom/TF output.
    topo_stack = TimerAction(
        period=15.0,
        actions=sowbot_sim._topo_nav_nodes(tmap2_file, devkit_launch_pkg, use_sim_time=True),
    )

    return LaunchDescription([
        odom_relay,
        odom_to_base_footprint,
        base_footprint_to_base_link,
        ui_node,
        fake_nav2,
        kill_fake_nav2,
        topo_stack,
    ])
