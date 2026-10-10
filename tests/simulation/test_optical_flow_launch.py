"""The optical flow driver must start with the project overrides and never own odom -> base_link."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
OVERRIDES = REPO_ROOT / 'src/devkit_bringup/config/optical_flow.yaml'


def load(launch_file, tmp_path):
    """Return the driver node and the other launch actions, plus the expected parameter files."""
    actions = launch_file('devkit_bringup', 'optical_flow.launch.py')
    nodes = [action for action in actions if action.kind == 'LifecycleNode']
    assert len(nodes) == 1
    shares = tmp_path / 'package shares'
    files = [
        str(shares / 'optical_flow_ros' / 'config' / 'sensor_params.yaml'),
        str(shares / 'devkit_bringup' / 'config' / 'optical_flow.yaml'),
    ]
    return nodes[0], [action for action in actions if action is not nodes[0]], files


def test_driver_loads_overrides_after_sensor_parameters(launch_file, tmp_path):
    """Later parameter files win, so the override file must come last."""
    node, _, files = load(launch_file, tmp_path)
    assert node.kwargs['parameters'] == files
    assert node.kwargs['package'] == 'optical_flow_ros'
    assert node.kwargs['executable'] == 'optical_flow_publisher'
    assert node.kwargs['name'] == 'optical_flow'
    assert node.kwargs['namespace'] == ''
    assert node.kwargs['remappings'] == [('odom', 'flow_odom')]


def test_override_file_only_disables_the_transform():
    """The override must be a real boolean for the driver's node name, and change nothing else."""
    assert yaml.safe_load(OVERRIDES.read_text()) == {'optical_flow': {'ros__parameters': {'publish_tf': False}}}


def test_lifecycle_node_is_configured_then_activated_once(launch_file, tmp_path):
    """Configure at start; activate only after configuring reaches inactive."""
    node, others, _ = load(launch_file, tmp_path)
    emits = [action for action in others if action.kind == 'EmitEvent']
    assert [emit.kwargs['event'].kwargs['transition_id'] for emit in emits] == [1]
    assert emits[0].kwargs['event'].kwargs['lifecycle_node_matcher'].args == (node,)

    handlers = [action.args[0] for action in others if action.kind == 'RegisterEventHandler']
    assert len(handlers) == 1
    assert handlers[0].kwargs['target_lifecycle_node'] is node
    assert handlers[0].kwargs['start_state'] == 'configuring'
    assert handlers[0].kwargs['goal_state'] == 'inactive'
    activations = handlers[0].kwargs['entities']
    assert [entity.kwargs['event'].kwargs['transition_id'] for entity in activations] == [3]
    assert activations[0].kwargs['event'].kwargs['lifecycle_node_matcher'].args == (node,)
