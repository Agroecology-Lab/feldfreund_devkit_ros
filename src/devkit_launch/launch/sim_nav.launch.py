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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

# Re-import the shared helper functions from sowbot_sim so the Nav2 node
# list and topo-nav node list stay in one place.
# sowbot_sim.launch.py lives in the same package directory; we load it via
# importlib so launch infrastructure doesn't need to know about the import.
import importlib.util, pathlib

def _load_sowbot_sim():
    pkg = get_package_share_directory('devkit_launch')
    path = pathlib.Path(pkg) / 'launch' / 'sowbot_sim.launch.py'
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
        output='screen',
    )

    # ── Static map → odom TF ──────────────────────────────────────────────────
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
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

    # ── Topo nav + Nav2 (delayed until Gazebo sim layer is up) ───────────────
    # Gazebo may not have started yet when this boots — the 5 s timer matches
    # sowbot_sim.launch.py so that if the user starts Gazebo immediately the
    # stack is ready; if Gazebo starts later Nav2/topo-nav will wait for TF.
    topo_stack = TimerAction(
        period=5.0,
        actions=sowbot_sim._topo_nav_nodes(tmap2_file, devkit_launch_pkg),
    )

    return LaunchDescription([
        odom_relay,
        map_to_odom,
        ui_node,
        topo_stack,
    ])
