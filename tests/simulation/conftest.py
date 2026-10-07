"""Capture launch declarations without importing ROS or starting processes."""

import importlib.util
import os
import subprocess
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def launch_file(monkeypatch, tmp_path):
    """Load real launch code with passive doubles for the ROS launch API."""
    exports = {
        'ament_index_python.packages': 'get_package_share_directory',
        'launch': 'LaunchDescription',
        'launch.actions': (
            'DeclareLaunchArgument ExecuteProcess RegisterEventHandler TimerAction '
            'EmitEvent IncludeLaunchDescription LogInfo SetEnvironmentVariable'
        ),
        'launch.conditions': 'IfCondition UnlessCondition',
        'launch.event_handlers': 'OnProcessExit',
        'launch.events': 'matches_action',
        'launch.launch_description_sources': 'PythonLaunchDescriptionSource',
        'launch.substitutions': 'Command LaunchConfiguration PathJoinSubstitution',
        'launch_ros.actions': 'Node LifecycleNode',
        'launch_ros.parameter_descriptions': 'ParameterValue',
        'launch_ros.events.lifecycle': 'ChangeState',
        'launch_ros.substitutions': 'FindPackageShare',
        'lifecycle_msgs.msg': 'Transition',
    }
    modules = {}
    for module_name in exports:
        parts = module_name.split('.')
        for size in range(1, len(parts) + 1):
            name = '.'.join(parts[:size])
            modules.setdefault(name, types.ModuleType(name))
    for name, module in modules.items():
        # NOTE: Replace even installed modules, then restore them with monkeypatch.
        monkeypatch.setitem(sys.modules, name, module)
    for module_name, names in exports.items():
        module = modules[module_name]
        for name in names.split():
            setattr(module, name, type(name, (LaunchEntity,), {}))
    share = tmp_path / 'package shares'
    sys.modules['ament_index_python.packages'].get_package_share_directory = (
        lambda package: str(share / package)
    )
    sys.modules['lifecycle_msgs.msg'].Transition.TRANSITION_CONFIGURE = 1

    def load(package, filename):
        """Return top-level actions from a launch file loaded with the ROS doubles."""
        path = REPO_ROOT / 'src' / package / 'launch' / filename
        spec = importlib.util.spec_from_file_location(f'test_{package}_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        # NOTE: The spawn script uses this executable; a shell function replaces Gazebo.
        module.GZ_BIN = 'gz'
        return module.generate_launch_description().args[0]

    return load


@pytest.fixture
def run_shell(tmp_path):
    """Run generated Bash with only explicitly supplied fake external commands."""
    def run(command, functions, expected_returncode=0, **environment):
        """Execute Bash with fake commands and assert its expected exit status."""
        result = subprocess.run(
            [command[0], command[1], functions + '\n' + command[2], *command[3:]],
            cwd=tmp_path,
            env={'PATH': '', **environment},
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        assert result.returncode == expected_returncode, result.stderr
        return result

    return run


class LaunchEntity:
    """Record constructor inputs; implement only substitutions used by these tests."""

    def __init__(self, *args, **kwargs):
        """Capture a launch entity's arguments and concrete API type name."""
        self.args = args
        self.kwargs = kwargs
        self.kind = type(self).__name__

    def resolve(self, configuration):
        """Evaluate supported substitutions, raising on an unsupported entity kind."""
        if self.kind == 'LaunchConfiguration':
            return configuration[self.args[0]]
        if self.kind == 'PathJoinSubstitution':
            return os.path.join(*(resolve(part, configuration) for part in self.args[0]))
        if self.kind == 'Command':
            return ''.join(resolve(part, configuration) for part in self.args[0])
        raise AssertionError(f'Unsupported substitution: {self.kind}')


def resolve(value, configuration):
    """Resolve a launch substitution or return a literal value unchanged."""
    return value.resolve(configuration) if isinstance(value, LaunchEntity) else value


@pytest.fixture
def launch_helpers():
    """Provide helpers for inspecting captured launch actions and substitutions."""
    def defaults(actions):
        """Map declared launch argument names to their default values."""
        return {
            action.args[0]: action.kwargs['default_value']
            for action in actions if action.kind == 'DeclareLaunchArgument'
        }

    def named(actions, name):
        """Return the uniquely named action, asserting exactly one match exists."""
        matches = [action for action in actions if action.kwargs.get('name') == name]
        assert len(matches) == 1, (name, matches)
        return matches[0]

    def handlers_for(actions, target_name):
        """Return registered event handlers targeting the named action."""
        return [
            action.args[0] for action in actions
            if action.kind == 'RegisterEventHandler'
            and action.args[0].kwargs['target_action'].kwargs.get('name') == target_name
        ]

    def worldgen_handlers(actions):
        """Return the event handlers attached to world generation."""
        return handlers_for(actions, 'world_gen')

    return types.SimpleNamespace(
        defaults=defaults, named=named, worldgen_handlers=worldgen_handlers,
        handlers_for=handlers_for, resolve=resolve,
    )
