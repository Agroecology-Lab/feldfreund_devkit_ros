"""Exercise optical-flow Docker RUN commands without Docker, network, or hardware."""

import os
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SENSOR_CONFIG = Path('src/optical_flow_ros/config/sensor_params.yaml')
DRIVER_URL = 'https://github.com/adityakamath/optical_flow_ros'
DRIVER_COMMIT = '423965e74a7282371ffaf1d0d2981895c561f0aa'


def test_driver_commit_is_pinned(docker_instructions):
    """Rebuilding must select the reviewed upstream revision, not a moving branch."""
    assert build_args(docker_instructions)['OPTICAL_FLOW_ROS_COMMIT'] == DRIVER_COMMIT


@pytest.mark.parametrize('pip_status', [0, 1])
def test_sensor_dependencies_are_installed_and_failures_propagate(
    docker_instructions, tmp_path, pip_status,
):
    """Install all missing hardware dependencies and reject a failed installation."""
    command = run_instruction(docker_instructions, 'pmw3901')
    result = run_shell(command, tmp_path, prelude=r'''
pip() { printf '%s\n' "$@" > pip.args; return "$PIP_STATUS"; }
''', PIP_STATUS=str(pip_status))

    assert result.returncode == pip_status, result.stderr
    args = (tmp_path / 'pip.args').read_text().splitlines()
    assert args[0] == 'install'
    assert '--break-system-packages' in args
    assert {'pmw3901', 'RPi.GPIO', 'gpiod', 'gpiodevice'} <= set(args)


def test_optical_flow_layers_precede_workspace_build(docker_instructions):
    """Dependencies and patched source must exist before rosdep and colcon run."""
    dependencies = run_instruction(docker_instructions, 'pmw3901')
    clone = run_instruction(docker_instructions, DRIVER_URL)
    patch = run_instruction(docker_instructions, str(SENSOR_CONFIG))
    rosdep = run_instruction(docker_instructions, 'rosdep install')
    colcon = run_instruction(docker_instructions, 'colcon build')
    commands = [shell_command(line) for line in docker_instructions if line.startswith('RUN ')]
    assert commands.index(dependencies) < commands.index(clone)
    assert commands.index(clone) < commands.index(patch) < commands.index(rosdep)
    assert commands.index(rosdep) < commands.index(colcon)


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


@pytest.mark.parametrize('initial_value', ['true', 'false'])
def test_tf_patch_disables_broadcast_and_preserves_sensor_settings(
    docker_instructions, tmp_path, initial_value,
):
    """Keep FusionCore as TF owner, preserving parameters and allowing repeated builds."""
    config = tmp_path / SENSOR_CONFIG
    config.parent.mkdir(parents=True)
    prefix = (
        'optical_flow:\n'
        '    ros__parameters:\n'
        '        parent_frame: odom\n'
        '        child_frame: base_link\n'
        '        board: paa5100\n'
        '        spi_nr: 0\n'
        '        spi_slot: front\n'
        '        rotation: 0\n'
        '        z_height: 0.025\n'
    )
    config.write_text(prefix + f'        publish_tf: {initial_value}\n')
    unrelated = config.with_name('other_params.yaml')
    unrelated.write_text('publish_tf: true\n')
    command = run_instruction(docker_instructions, str(SENSOR_CONFIG))

    for _ in range(2):
        result = run_shell(command, tmp_path)
        assert result.returncode == 0, result.stderr
        assert config.read_text() == prefix + '        publish_tf: false\n'
    assert unrelated.read_text() == 'publish_tf: true\n'


@pytest.mark.parametrize('contents', [None, '', 'optical_flow:\n    ros__parameters: {}\n',
                                     'optical_flow:\n    ros__parameters:\n        publish_tf: yes\n'])
def test_tf_patch_rejects_missing_or_unrecognized_setting(
    docker_instructions, tmp_path, contents,
):
    """Fail the build when upstream drift prevents the expected TF configuration."""
    config = tmp_path / SENSOR_CONFIG
    config.parent.mkdir(parents=True)
    if contents is not None:
        config.write_text(contents)

    result = run_shell(run_instruction(docker_instructions, str(SENSOR_CONFIG)), tmp_path)

    assert result.returncode != 0
    if contents is None:
        assert not config.exists()
    else:
        assert config.read_text() == contents


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
