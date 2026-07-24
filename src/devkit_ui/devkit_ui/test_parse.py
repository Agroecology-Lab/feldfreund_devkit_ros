import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from deepdiff import DeepDiff

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


if __name__ == '__main__':
    unittest.main()
