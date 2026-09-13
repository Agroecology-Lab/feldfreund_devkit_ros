import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from deepdiff import DeepDiff

from devkit_ui.constants import NAV_ACTION, ROW_ACTION
from devkit_ui.models import TopoDoc, TopoNode
from devkit_ui.parse import dump_topo_yaml, parse_topo_yaml


class TestParseTopo(unittest.TestCase):
    def test_dump_and_restore_yaml_round_trip(self) -> None:
        original = TopoDoc(
            name='test-topo',
            nodes=[
                TopoNode(name='A', x=0.0, y=0.0, edges=['B'], meta={'kind': 'start'}),
                TopoNode(name='B', x=1.0, y=1.0, edges=['A', 'C']),
                TopoNode(name='C', x=2.0, y=2.0, edges=['B']),
            ],
            meta={'map': 'field-1'},
            actions={'move_base': {}},
            definitions={'robot': {'type': 'tractor'}},
            transformation={'frame': 'map'},
        )

        with TemporaryDirectory() as tmp_dir:
            yaml_path = Path(tmp_dir) / 'topo.yaml'
            dump_topo_yaml(original, str(yaml_path))
            restored = parse_topo_yaml(str(yaml_path))

        original_dict = original.to_dict()
        restored_dict = restored.to_dict()
        # `dump_topo_yaml` injects a timestamp on write, so ignore it for equality.
        original_dict.get('meta', {}).pop('last_updated', None)
        restored_dict.get('meta', {}).pop('last_updated', None)

        diff = DeepDiff(restored_dict, original_dict)
        self.assertDictEqual(diff, {})

    def test_edge_action_survives_yaml_round_trip(self) -> None:
        """A non-empty edge action (e.g. limbic_row_follow) must not be
        silently dropped to '' on load. The previous test above only ever
        round-tripped edges built via edges=['B'] (plain strings), which
        already default to action='' in TopoNode.__init__ — so it never
        exercised a non-empty action and couldn't catch this regression."""
        a = TopoNode(name='A', x=0.0, y=0.0, edges=[])
        a.add_edge('B', action=ROW_ACTION)
        a.add_edge('C', action=NAV_ACTION)
        original = TopoDoc(
            name='test-topo-actions',
            nodes=[a, TopoNode(name='B', x=1.0, y=1.0, edges=[]),
                   TopoNode(name='C', x=2.0, y=2.0, edges=[])],
        )

        with TemporaryDirectory() as tmp_dir:
            yaml_path = Path(tmp_dir) / 'topo.yaml'
            dump_topo_yaml(original, str(yaml_path))
            restored = parse_topo_yaml(str(yaml_path))

        actions = {e.node: e.action for e in restored.get_node('A').edges}
        self.assertEqual(actions, {'B': ROW_ACTION, 'C': NAV_ACTION})

        # A second round trip (mirrors _persist_and_reload's load -> modify
        # -> save cycle on every subsequent map edit) must not corrupt it
        # either.
        with TemporaryDirectory() as tmp_dir:
            yaml_path = Path(tmp_dir) / 'topo2.yaml'
            dump_topo_yaml(restored, str(yaml_path))
            restored_again = parse_topo_yaml(str(yaml_path))

        actions_again = {e.node: e.action for e in restored_again.get_node('A').edges}
        self.assertEqual(actions_again, {'B': ROW_ACTION, 'C': NAV_ACTION})


if __name__ == '__main__':
    unittest.main()
