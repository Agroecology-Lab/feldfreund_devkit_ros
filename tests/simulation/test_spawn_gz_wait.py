"""The spawn command must stop waiting for Gazebo after a deadline."""

import pytest

# `sleep` advances bash's SECONDS so the deadline passes without real waiting.
FAKES = r'''
sleep() { SECONDS=$((SECONDS + $1)); echo "$1" >> sleeps; }
grep() { local line; read -r line; [[ "$line" == /world/* ]]; }
xacro() { printf '<robot name="r"/>'; }
ros2() { echo called > ros2.called; }
'''
WORLD_UP = 'gz() { printf "/world/test\\n"; }\n'
WORLD_NEVER = 'gz() { :; }\n'


def _spawn_command(launch_file, launch_helpers):
    actions = launch_file('devkit_simulation', 'sim.launch.py')
    configuration = launch_helpers.defaults(actions)
    spawn = launch_helpers.named(actions, 'spawn_robot')
    return [launch_helpers.resolve(part, configuration) for part in spawn.kwargs['cmd']]


def _sleeps(tmp_path):
    path = tmp_path / 'sleeps'
    return path.read_text().split() if path.exists() else []


def test_missing_world_times_out_with_status_1_and_no_spawn(
    launch_file, launch_helpers, run_shell, tmp_path,
):
    """Verify a gz that never reports a world ends the spawn after 180 s without creating a robot."""
    command = _spawn_command(launch_file, launch_helpers)
    result = run_shell(command, FAKES + WORLD_NEVER, expected_returncode=1)
    assert 'no world within 180s' in result.stderr
    assert len(_sleeps(tmp_path)) == 90
    assert not (tmp_path / 'ros2.called').exists()


def test_timeout_is_configurable(launch_file, launch_helpers, run_shell, tmp_path):
    """Verify SPAWN_GZ_TIMEOUT_S overrides the default deadline."""
    command = _spawn_command(launch_file, launch_helpers)
    result = run_shell(
        command, FAKES + WORLD_NEVER, expected_returncode=1, SPAWN_GZ_TIMEOUT_S='10',
    )
    assert 'no world within 10s' in result.stderr
    assert len(_sleeps(tmp_path)) == 5


def test_world_already_up_does_not_wait_or_time_out(
    launch_file, launch_helpers, run_shell, tmp_path,
):
    """Verify a ready gz skips the wait loop and still reaches the spawn call."""
    command = _spawn_command(launch_file, launch_helpers)
    run_shell(command, FAKES + WORLD_UP, expected_returncode=0)
    # Only the 30 s GUI-init sleep remains; the 2 s wait-loop sleep is never taken.
    assert _sleeps(tmp_path) == ['30']
    assert (tmp_path / 'ros2.called').exists()


@pytest.mark.parametrize('ready_after', [1, 5])
def test_world_appearing_before_deadline_spawns(
    launch_file, launch_helpers, run_shell, tmp_path, ready_after,
):
    """Verify a gz that comes up late but in time proceeds to spawn the robot."""
    command = _spawn_command(launch_file, launch_helpers)
    flaky_gz = (
        # PATH is empty and gz runs in a pipeline subshell, so count polls in a file with builtins.
        'gz() { local n=0; [[ -f polls ]] && read -r n < polls; echo $((n + 1)) > polls; '
        f'if (( n >= {ready_after} )); then printf "/world/test\\n"; fi; }}\n'
    )
    run_shell(command, FAKES + flaky_gz, expected_returncode=0)
    assert _sleeps(tmp_path)[:ready_after] == ['2'] * ready_after
    assert (tmp_path / 'ros2.called').exists()
