"""
nav2_only.launch.py
====================
Nav2 server nodes + lifecycle manager, with NOTHING else — no Gazebo,
no robot spawn, no ros_gz_bridge, no preflight_pkill.

Why this file exists
---------------------
sowbot_sim.launch.py's sim_launch step (IncludeLaunchDescription of
sim.launch.py) unconditionally starts its own gz sim + robot_state_publisher
+ spawn_entity + bridge, and its preflight_pkill step kills any existing
"gz sim"/ros_gz_sim/parameter_bridge process first. That's correct for the
browser flow (Launch Sim (browser) in the UI), which owns the whole stack
from a clean slate — but it makes sowbot_sim.launch.py unsafe to chain after
the UI's manual Launch World + Spawn Robot buttons, which already started
Gazebo/the robot/the bridge by hand: chaining sowbot_sim.launch.py there
would kill what's already running and spawn a duplicate world+robot
underneath it.

This file extracts ONLY the Nav2 piece so the manual flow can add Nav2 on
top of an already-running sim without touching Gazebo/spawn/bridge at all.

_nav2_sim_nodes() and _nav2_lifecycle_manager_node() are imported directly
from sowbot_sim.launch.py via importlib (same install dir, see below) so
there is exactly one definition of Nav2's params/remappings/lifecycle-stagger
fix — used by sowbot_sim.launch.py, this file, AND (if ever needed) any
future caller. Do not copy these functions; import them.

fake_nav2_server handoff
-------------------------
fake_nav2_server (started always-on at container boot by sim_nav.launch.py)
publishes a wall-time /clock so Nav2/topo-nav's use_sim_time=True nodes have
a valid clock before Gazebo exists — without it, TF lookups fail with
"Extrapolation Error" from boot until Gazebo's bridge comes up. We keep that
behaviour. But its NavigateToPose/NavigateThroughPoses/FollowWaypoints
action servers must NOT keep running once real Nav2's bt_navigator starts —
two servers on the same action name is a collision, and we want real Nav2
to always be the actual navigation backend now (in sim as well as on real
hardware). So kill_fake_nav2 below kills the fake_nav2_server process the
moment this file runs — i.e. the moment any UI path actually brings Gazebo
up. Until that point fake_nav2_server keeps the clock/TF alive exactly as
before; this is a real-Nav2 takeover, not a fake_nav2_server removal.

Timing
------
Unlike sowbot_sim.launch.py's nav2 (t+15s) / nav2_lifecycle (t+20s), which
wait for Gazebo+bridge to come up from a cold start, this file assumes the
caller already has /clock live (Gazebo/bridge were started separately,
before this launch file runs) — so Nav2 starts immediately (t+0s) and the
lifecycle manager keeps its 5s stagger relative to that, not to Gazebo's
boot time. If Nav2 still comes up before /clock is reliably publishing in
your environment, increase nav2_start_delay below; don't remove the 5s
stagger between nav2 and nav2_lifecycle — see _nav2_lifecycle_manager_node's
docstring in sowbot_sim.launch.py for why that gap matters.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def _import_sowbot_sim_launch():
    """Load sowbot_sim.launch.py as a module so we can reuse its Nav2 node
    helpers without copying them. Both files live in the same installed
    launch/ directory (see devkit_bringup/CMakeLists.txt — the whole launch
    folder is installed as one unit), so this path is stable."""
    this_dir = os.path.dirname(os.path.realpath(__file__))
    sowbot_sim_path = os.path.join(this_dir, 'sowbot_sim.launch.py')
    spec = importlib.util.spec_from_file_location('sowbot_sim_launch', sowbot_sim_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    sowbot_sim = _import_sowbot_sim_launch()
    devkit_launch_pkg = get_package_share_directory('devkit_bringup')
    nav2_params = os.path.join(devkit_launch_pkg, 'config', 'nav2_params_sim.yaml')

    # t+0s: kill the boot-time fake_nav2_server. Its /clock+TF stub job is
    # done — real Nav2 (started below) is about to take over both the
    # navigation backend and, once ros_gz_bridge's RELIABLE /clock starts
    # publishing, the clock source too. Leaving fake_nav2_server alive past
    # this point would mean two NavigateToPose action servers competing for
    # the same action name. `|| true` so this is a no-op if fake_nav2_server
    # was already stopped (e.g. nav2_only.launch.py run a second time).
    kill_fake_nav2 = ExecuteProcess(
        cmd=['/bin/bash', '-c', 'pkill -f fake_nav2_server || true'],
        name='kill_fake_nav2',
        output='screen',
    )

    # t+0s: Nav2 server nodes. No startup delay here — the caller is
    # responsible for ensuring Gazebo/bridge/clock are already up before
    # including this file (see module docstring).
    nav2_start_delay = 0.0
    nav2 = TimerAction(
        period=nav2_start_delay,
        actions=sowbot_sim._nav2_sim_nodes(nav2_params, use_sim_time=True),  # pylint: disable=protected-access
    )

    # t+5s: lifecycle manager, staggered after the nodes above — same gap
    # and same bond_timeout as sowbot_sim.launch.py, just relative to this
    # file's own start time instead of Gazebo's boot time.
    nav2_lifecycle = TimerAction(
        period=nav2_start_delay + 5.0,
        actions=[sowbot_sim._nav2_lifecycle_manager_node(  # pylint: disable=protected-access
            nav2_params, use_sim_time=True)],
    )

    return LaunchDescription([
        kill_fake_nav2,
        nav2,
        nav2_lifecycle,
    ])
