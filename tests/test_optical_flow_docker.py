"""Exercise optical-flow Docker RUN commands without Docker, network, or hardware."""

import os
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER_URL = 'https://github.com/adityakamath/optical_flow_ros'
DRIVER_COMMIT = '423965e74a7282371ffaf1d0d2981895c561f0aa'
SENSOR_PACKAGES = {'pmw3901==1.0.0', 'gpiod==2.5.0', 'gpiodevice==0.1.0'}


def test_driver_commit_is_pinned(docker_instructions):
    """Rebuilding must select the reviewed upstream revision, not a moving branch."""
    assert build_args(docker_instructions)['OPTICAL_FLOW_ROS_COMMIT'] == DRIVER_COMMIT


@pytest.mark.parametrize('pip_status', [0, 1])
def test_sensor_dependencies_are_pinned_and_failures_propagate(
    docker_instructions, tmp_path, pip_status,
):
    """Install exactly the pinned hardware dependencies and reject a failed installation."""
    command = run_instruction(docker_instructions, 'pmw3901')
    result = run_shell(command, tmp_path, prelude=r'''
pip() { printf '%s\n' "$@" > pip.args; return "$PIP_STATUS"; }
''', PIP_STATUS=str(pip_status))

    assert result.returncode == pip_status, result.stderr
    args = (tmp_path / 'pip.args').read_text().splitlines()
    assert args[0] == 'install'
    assert '--break-system-packages' in args
    # NOTE: RPi.GPIO is imported by neither pmw3901 1.0.0 nor optical_flow_ros.
    assert set(args[1:]) - {'--break-system-packages'} == SENSOR_PACKAGES


def test_optical_flow_layers_precede_workspace_build(docker_instructions):
    """Dependencies and source must exist before rosdep and colcon run."""
    dependencies = run_instruction(docker_instructions, 'pmw3901')
    clone = run_instruction(docker_instructions, DRIVER_URL)
    rosdep = run_instruction(docker_instructions, 'rosdep install')
    colcon = run_instruction(docker_instructions, 'colcon build')
    commands = [shell_command(line) for line in docker_instructions if line.startswith('RUN ')]
    assert commands.index(dependencies) < commands.index(clone)
    assert commands.index(clone) < commands.index(rosdep) < commands.index(colcon)


def test_driver_clone_is_not_patched(docker_instructions):
    """TF ownership is set by devkit_bringup/config/optical_flow.yaml, so one mechanism decides."""
    assert not [line for line in docker_instructions
                if line.startswith('RUN ') and 'src/optical_flow_ros/' in line]


@pytest.mark.parametrize('commit_override', [None, '0123456789abcdef0123456789abcdef01234567'])
def test_driver_clone_uses_requested_revision(docker_instructions, tmp_path, commit_override):
    """Fetch the default or overridden pin and check out FETCH_HEAD in the workspace."""
    args = build_args(docker_instructions)
    if commit_override is not None:
        args['OPTICAL_FLOW_ROS_COMMIT'] = commit_override
    result = clone_workspace(docker_instructions, tmp_path, args)

    assert result.returncode == 0, result.stderr
    assert (tmp_path / 'optical-git.log').read_text().splitlines() == [
        'init -q',
        f'fetch --depth 1 {DRIVER_URL} {commit_override or DRIVER_COMMIT}',
        'prompt=0',
        'checkout -q FETCH_HEAD',
    ]
    assert (tmp_path / 'src/optical_flow_ros/checked-out').is_file()


@pytest.mark.parametrize('failed_operation', ['fetch', 'checkout'])
@pytest.mark.parametrize('failed_attempts', [1, 4, 5])
def test_driver_clone_retries_and_propagates_exhaustion(
    docker_instructions, tmp_path, failed_operation, failed_attempts,
):
    """An optical-flow failure must not be hidden by the other successful clones."""
    result = clone_workspace(
        docker_instructions, tmp_path, build_args(docker_instructions),
        FAIL_OPERATION=failed_operation, FAIL_ATTEMPTS=str(failed_attempts),
    )
    exhausted = failed_attempts == 5
    assert (result.returncode != 0) is exhausted, result.stderr
    assert int((tmp_path / 'attempts').read_text()) == min(failed_attempts + 1, 5)
    destination = tmp_path / 'src/optical_flow_ros'
    if exhausted:
        assert not destination.exists()
        assert f'FATAL: pinclone failed: {DRIVER_URL}@{DRIVER_COMMIT}' in result.stdout
    else:
        assert (destination / 'checked-out').is_file()
        assert not (destination / 'partial-fetch').exists()


@pytest.fixture(scope='module')
def docker_instructions():
    """Join Docker continuation lines, excluding comments from command selection."""
    source = '\n'.join(line for line in (ROOT / 'docker/Dockerfile').read_text().splitlines()
                       if not line.lstrip().startswith('#'))
    return [line.strip() for line in re.sub(r'\\\n', ' ', source).splitlines()
            if line.strip()]


def shell_command(instruction):
    """Remove Docker's RUN and BuildKit mount options before invoking the shell."""
    return re.sub(r'^RUN\s+(?:--mount=\S+\s+)*', '', instruction)


def run_instruction(instructions, marker):
    matches = [shell_command(line) for line in instructions
               if line.startswith('RUN ') and marker in line]
    assert len(matches) == 1, f'Expected one RUN containing {marker!r}, found {len(matches)}'
    return matches[0]


def build_args(instructions):
    return dict(line[4:].split('=', 1) for line in instructions
                if line.startswith('ARG ') and '_COMMIT=' in line)


def run_shell(command, directory, prelude='', **environment):
    return subprocess.run(
        ['/bin/sh', '-c', prelude + '\n' + command],
        cwd=directory, env={**os.environ, **environment},
        capture_output=True, text=True, timeout=5, check=False,
    )


def clone_workspace(instructions, directory, args, **environment):
    """Run the real parallel clone layer with offline Git and immediate retry doubles."""
    sentinel = directory / 'src/vision_opencv/cv_bridge/package.xml'
    sentinel.parent.mkdir(parents=True)
    sentinel.touch()
    settings = {'FAIL_OPERATION': '', 'FAIL_ATTEMPTS': '0', **environment}
    return run_shell(
        run_instruction(instructions, DRIVER_URL), directory, prelude=GIT_DOUBLE,
        **args, TEST_WORKSPACE=str(directory), **settings,
    )


GIT_DOUBLE = r'''
git() {
    case "$PWD" in
        "$TEST_WORKSPACE/src/optical_flow_ros") ;;
        *) return 0 ;;
    esac
    printf '%s\n' "$*" >> "$TEST_WORKSPACE/optical-git.log"
    attempt=0
    if [ -f "$TEST_WORKSPACE/attempts" ]; then
        read -r attempt < "$TEST_WORKSPACE/attempts"
    fi
    if [ "$1" = init ]; then
        attempt=$((attempt + 1))
        printf '%s\n' "$attempt" > "$TEST_WORKSPACE/attempts"
    fi
    if [ "$1" = fetch ]; then
        printf 'prompt=%s\n' "$GIT_TERMINAL_PROMPT" >> "$TEST_WORKSPACE/optical-git.log"
    fi
    if [ "$1" = "$FAIL_OPERATION" ] && [ "$attempt" -le "$FAIL_ATTEMPTS" ]; then
        touch partial-fetch
        return 1
    fi
    if [ "$1" = checkout ]; then touch checked-out; fi
    return 0
}
sleep() { :; }
'''
