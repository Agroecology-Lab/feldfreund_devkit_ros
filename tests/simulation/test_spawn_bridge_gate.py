"""The ROS-Gazebo bridge must start only after a successful robot spawn."""

from types import SimpleNamespace

import pytest


def _spawn_handlers(launch_file, launch_helpers):
    actions = launch_file('devkit_simulation', 'sim.launch.py')
    return actions, launch_helpers.handlers_for(actions, 'spawn_robot')


def test_spawn_success_starts_delayed_bridge(launch_file, launch_helpers):
    """Verify a zero spawn status releases only the bridge, two seconds later."""
    actions, handlers = _spawn_handlers(launch_file, launch_helpers)
    assert len(handlers) == 1
    released = handlers[0].kwargs['on_exit'](SimpleNamespace(returncode=0), None)
    assert len(released) == 1 and released[0].kind == 'TimerAction'
    assert released[0].kwargs['period'] == 2.0
    nodes = released[0].kwargs['actions']
    assert [node.kwargs['name'] for node in nodes] == ['ros_gz_bridge']
    assert all(action not in actions for action in released)


@pytest.mark.parametrize('returncode', [1, 2, 127, -9, -15])
def test_spawn_failure_logs_and_starts_no_bridge(launch_file, launch_helpers, returncode):
    """Verify any non-zero spawn status logs the code and releases no bridge."""
    _, handlers = _spawn_handlers(launch_file, launch_helpers)
    released = handlers[0].kwargs['on_exit'](SimpleNamespace(returncode=returncode), None)
    assert len(released) == 1 and released[0].kind == 'LogInfo'
    assert str(returncode) in released[0].kwargs['msg']
    assert not any(action.kind in ('TimerAction', 'Node') for action in released)
