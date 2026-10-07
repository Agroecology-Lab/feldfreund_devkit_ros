"""Regression tests for world-generation and bootstrap-TF readiness gates."""

import re
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('returncode', [1, 2, 127, -9, -15])
def test_worldgen_failure_blocks_all_downstream_actions(
    launch_file, launch_helpers, returncode,
):
    """Verify failed world generation logs its status and releases no dependents."""
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    handlers = launch_helpers.worldgen_handlers(actions)
    assert len(handlers) == 3
    returned = [
        action
        for handler in handlers
        for action in handler.kwargs['on_exit'](SimpleNamespace(returncode=returncode), None)
    ]
    assert len(returned) == 1
    assert returned[0].kind == 'LogInfo'
    assert str(returncode) in returned[0].kwargs['msg']
    assert not any(action.kind in ('TimerAction', 'IncludeLaunchDescription') for action in actions)
    assert not any(action.kwargs.get('name') in ('map_to_odom_fixer', 'clock_gate')
                   for action in actions)


def test_worldgen_success_releases_sim_gate_and_fixer(launch_file, launch_helpers):
    """Verify successful world generation releases only the simulation, gate, and fixer."""
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    handlers = launch_helpers.worldgen_handlers(actions)
    assert len(handlers) == 3
    targets = [handler.kwargs['target_action'] for handler in handlers]
    assert all(target is targets[0] for target in targets)
    returned = [
        handler.kwargs['on_exit'](SimpleNamespace(returncode=0), None) for handler in handlers
    ]
    assert all(len(result) == 1 for result in returned)
    released = [result[0] for result in returned]
    assert sum(action.kind == 'IncludeLaunchDescription' for action in released) == 1
    # Nav2/fusioncore are NOT released by world_gen any more: only by the /clock gate.
    assert not any(action.kind == 'TimerAction' for action in released)
    assert launch_helpers.named(released, 'clock_gate').kind == 'ExecuteProcess'
    assert launch_helpers.named(released, 'map_to_odom_fixer').kind == 'ExecuteProcess'
    # NOTE: Every downstream action must remain gated, with no top-level duplicate.
    assert all(action not in actions for action in released)


@pytest.mark.parametrize('returncode', [1, 124, 127, -9])
def test_clock_gate_failure_releases_nothing(launch_file, launch_helpers, returncode):
    """Verify a failed clock gate leaves all dependent actions blocked."""
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    gate = launch_helpers.named(
        [a for a in actions if a.kind == 'ExecuteProcess'] + [
            r for h in launch_helpers.worldgen_handlers(actions)
            for r in h.kwargs['on_exit'](SimpleNamespace(returncode=0), None)
        ],
        'clock_gate',
    )
    handlers = [
        a.args[0] for a in actions
        if a.kind == 'RegisterEventHandler' and a.args[0].kwargs['target_action'] is gate
    ]
    assert len(handlers) == 4
    assert all(
        handler.kwargs['on_exit'](SimpleNamespace(returncode=returncode), None) == []
        for handler in handlers
    )


def test_clock_gate_success_releases_nav_and_fusioncore_timers(launch_file, launch_helpers):
    """Verify clock readiness schedules navigation and fusion nodes at the expected delays."""
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    gate = launch_helpers.named(
        [r for h in launch_helpers.worldgen_handlers(actions)
         for r in h.kwargs['on_exit'](SimpleNamespace(returncode=0), None)],
        'clock_gate',
    )
    handlers = [
        a.args[0] for a in actions
        if a.kind == 'RegisterEventHandler' and a.args[0].kwargs['target_action'] is gate
    ]
    assert len(handlers) == 4
    released = [
        action
        for handler in handlers
        for action in handler.kwargs['on_exit'](SimpleNamespace(returncode=0), None)
    ]
    assert len(released) == 4 and all(action.kind == 'TimerAction' for action in released)
    # Offsets are from /clock first seen: nodes and fusioncore at 5 s, lifecycle manager at 10 s.
    assert sorted(action.kwargs['period'] for action in released) == [5.0, 5.0, 5.0, 10.0]
    nodes = [node for timer in released for node in timer.kwargs['actions']]
    expected = {
        'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'lifecycle_manager_navigation',
        'fusioncore', 'fusion_odom_global_relay',
    }
    assert {node.kwargs['name'] for node in nodes if 'name' in node.kwargs} == expected
    assert all(action not in actions for action in released)


@pytest.mark.parametrize('wait_status', [0, 1, 124, 127])
def test_clock_gate_command_only_succeeds_when_clock_is_seen(
    launch_file, launch_helpers, run_shell, tmp_path, wait_status,
):
    """Verify the gate waits for /clock and succeeds only when that wait succeeds."""
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    gate = launch_helpers.named(
        [r for h in launch_helpers.worldgen_handlers(actions)
         for r in h.kwargs['on_exit'](SimpleNamespace(returncode=0), None)],
        'clock_gate',
    )
    result = run_shell(
        gate.kwargs['cmd'], TF_COMMANDS, expected_returncode=0 if wait_status == 0 else 1,
        WAIT_STATUS=str(wait_status), KILL_STATUS='0',
    )
    args = (tmp_path / 'wait.args').read_bytes().decode().rstrip('\0').split('\0')
    assert args == ['240', 'ros2', 'topic', 'echo', '/clock', '--once']
    if wait_status == 0:
        assert '/clock live' in result.stdout
    else:
        assert 'not starting Nav2/fusioncore' in result.stdout


@pytest.mark.parametrize('wait_status,kill_status', [(0, 0), (0, 1), (1, 0), (124, 0), (127, 0)])
def test_bootstrap_cleanup_requires_successful_dynamic_tf_wait(
    launch_file, launch_helpers, run_shell, tmp_path, wait_status, kill_status,
):
    """Verify cleanup targets only bootstrap base transforms after a successful TF wait."""
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    cleanup = launch_helpers.named(actions, 'kill_bootstrap_tfs')
    result = run_shell(
        cleanup.kwargs['cmd'], TF_COMMANDS,
        WAIT_STATUS=str(wait_status), KILL_STATUS=str(kill_status),
    )
    args = (tmp_path / 'wait.args').read_bytes().decode().rstrip('\0').split('\0')
    assert args[:6] == ['240', 'ros2', 'topic', 'echo', '/tf', '--once']
    assert args[6] == '--filter'
    assert len(args) == 8
    killed = tmp_path / 'kill.args'
    if wait_status == 0:
        patterns = killed.read_text().splitlines()
        assert len(patterns) == 2
        for name in ('odom_to_base_footprint_static', 'base_footprint_to_base_link_static'):
            process = (
                '/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher '
                f'--ros-args -r __node:={name}'
            )
            assert sum(bool(re.search(pattern, process)) for pattern in patterns) == 1
        for process in (
            'static_transform_publisher --ros-args -r __node:=map_to_odom_static',
            'fusioncore_node --ros-args -r __node:=fusioncore',
        ):
            assert not any(re.search(pattern, process) for pattern in patterns)
        assert not any(re.search(pattern, f'pkill -f {pattern}') for pattern in patterns)
    else:
        assert not killed.exists()
        assert 'keeping bootstrap statics' in result.stdout


@pytest.mark.parametrize('frames,expected', [
    ([], False),
    ([('odom', 'base_footprint')], True),
    ([('odom', 'base_link')], True),
    ([('map', 'odom')], False),
    ([('map', 'base_link')], False),
    ([('base_footprint', 'base_link')], False),
    ([('odom', 'camera_link')], False),
    ([('base_link', 'odom')], False),
    ([('map', 'base_link'), ('odom', 'camera_link')], False),
    ([('map', 'odom'), ('odom', 'base_footprint')], True),
])
def test_tf_filter_requires_matching_parent_and_child_in_same_transform(
    launch_file, launch_helpers, run_shell, tmp_path, frames, expected,
):
    """Verify the TF filter accepts only an odom-to-base pair within one transform."""
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    cleanup = launch_helpers.named(actions, 'kill_bootstrap_tfs')
    run_shell(cleanup.kwargs['cmd'], TF_COMMANDS, WAIT_STATUS='124', KILL_STATUS='0')
    args = (tmp_path / 'wait.args').read_bytes().decode().rstrip('\0').split('\0')
    expression = args[args.index('--filter') + 1]
    message = SimpleNamespace(transforms=[
        SimpleNamespace(header=SimpleNamespace(frame_id=parent), child_frame_id=child)
        for parent, child in frames
    ])
    # NOTE: Evaluate the actual expression sent to ros2 topic echo against synthetic TF messages.
    assert eval(expression, {'__builtins__': {}, 'any': any, 'm': message}) is expected


TF_COMMANDS = r'''
timeout() { printf '%s\0' "$@" > wait.args; return "$WAIT_STATUS"; }
pkill() {
    [[ "$1" == '-f' ]] || return 2
    printf '%s\n' "$2" >> kill.args
    return "$KILL_STATUS"
}
'''
