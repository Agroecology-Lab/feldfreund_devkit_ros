"""Camera settings must reach both robot descriptions through either entry point."""

import shlex
from pathlib import Path
from types import SimpleNamespace

import pytest

CAMERA_DEFAULTS = {
    'use_camera': 'true', 'camera_width': '320', 'camera_height': '240', 'camera_rate': '10',
}
OVERRIDES = [
    {},
    {'use_camera': 'false'},
    {'camera_width': '640', 'camera_height': '480', 'camera_rate': '29.97'},
    {'use_camera': 'false', 'camera_width': '1', 'camera_height': '2', 'camera_rate': '0.5'},
]


@pytest.mark.parametrize('package,filename', [
    ('devkit_simulation', 'sim.launch.py'),
    ('devkit_bringup', 'sowbot_sim.launch.py'),
])
def test_camera_defaults_at_both_entry_points(launch_file, launch_helpers, package, filename):
    actions = launch_file(package, filename)
    defaults = launch_helpers.defaults(actions)
    assert {key: defaults[key] for key in CAMERA_DEFAULTS} == CAMERA_DEFAULTS


@pytest.mark.parametrize('overrides', OVERRIDES)
def test_bringup_forwards_camera_configuration(launch_file, launch_helpers, overrides):
    actions = launch_file('devkit_bringup', 'sowbot_sim.launch.py')
    configuration = launch_helpers.defaults(actions) | overrides
    started = [
        action
        for handler in launch_helpers.worldgen_handlers(actions)
        for action in handler.kwargs['on_exit'](SimpleNamespace(returncode=0), None)
    ]
    includes = [action for action in started if action.kind == 'IncludeLaunchDescription']
    assert len(includes) == 1
    forwarded = dict(includes[0].kwargs['launch_arguments'])
    assert {
        name: launch_helpers.resolve(forwarded[name], configuration) for name in CAMERA_DEFAULTS
    } == CAMERA_DEFAULTS | overrides


@pytest.mark.parametrize('overrides', OVERRIDES)
def test_state_publisher_xacro_receives_camera_settings(launch_file, launch_helpers, overrides):
    actions = launch_file('devkit_simulation', 'sim.launch.py')
    configuration = launch_helpers.defaults(actions) | overrides
    publisher = launch_helpers.named(actions, 'robot_state_publisher')
    description = publisher.kwargs['parameters'][0]['robot_description']
    assert description.kwargs['value_type'] is str
    command = launch_helpers.resolve(description.args[0], configuration)
    # NOTE: Package share paths deliberately contain spaces; inspect the mapping suffix.
    mappings = shlex.split(command[command.index(' use_camera:='):])
    assert mappings == [f'{name}:={configuration[name]}' for name in CAMERA_DEFAULTS]


@pytest.mark.parametrize('overrides', OVERRIDES)
def test_spawn_xacro_receives_camera_settings(
    launch_file, launch_helpers, run_shell, tmp_path, overrides,
):
    actions = launch_file('devkit_simulation', 'sim.launch.py')
    configuration = launch_helpers.defaults(actions) | overrides
    spawn = launch_helpers.named(actions, 'spawn_robot')
    command = [launch_helpers.resolve(part, configuration) for part in spawn.kwargs['cmd']]
    run_shell(command, SPAWN_COMMANDS, DEVKIT_URDF='custom robot.xacro', XACRO_STATUS='0')
    args = read_arguments(tmp_path / 'xacro.args')
    assert Path(args[0]).name == 'custom robot.xacro'
    assert Path(args[0]).parent.name == 'urdf'
    assert args[1:] == [f'{name}:={configuration[name]}' for name in CAMERA_DEFAULTS]
    created = read_arguments(tmp_path / 'ros2.args')
    assert created[:3] == ['run', 'ros_gz_sim', 'create']
    assert created[created.index('-string') + 1] == '<robot name="camera test"/>'


def test_failed_xacro_does_not_spawn_robot(launch_file, launch_helpers, tmp_path, run_shell):
    actions = launch_file('devkit_simulation', 'sim.launch.py')
    configuration = launch_helpers.defaults(actions)
    command = [
        launch_helpers.resolve(part, configuration)
        for part in launch_helpers.named(actions, 'spawn_robot').kwargs['cmd']
    ]
    # NOTE: Preserve the generated command's status while making the harness exit successfully.
    command[2] += '\nstatus=$?; printf "%s" "$status" > spawn.status'
    run_shell(command, SPAWN_COMMANDS, XACRO_STATUS='2')
    assert (tmp_path / 'spawn.status').read_text() == '2'
    assert not (tmp_path / 'ros2.args').exists()


SPAWN_COMMANDS = r'''
gz() { printf '/world/test\n'; }
grep() { local line; read -r line; [[ "$line" == /world/* ]]; }
sleep() { :; }
xacro() {
    printf '%s\0' "$@" > xacro.args
    printf '<robot name="camera test"/>'
    return "$XACRO_STATUS"
}
ros2() { printf '%s\0' "$@" > ros2.args; }
'''


def read_arguments(path):
    return path.read_bytes().decode().rstrip('\0').split('\0')
