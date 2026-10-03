# pylint: disable=exec-used,no-member,protected-access,attribute-defined-outside-init
import ast
import copy
import re
import traceback
import unittest
from datetime import UTC, datetime
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock

from devkit_ui.constants import NAV_ACTION, ROW_ACTION
from devkit_ui.models import TopoDoc, TopoEdge, TopoNode, TopoProperties, Vector2
from devkit_ui.parse import dump_topo_yaml, parse_topo_yaml
from devkit_ui.topo_defaults import default_actions, default_definitions
from devkit_ui.view_models.run_view_model import RunViewModel


def load_drop_harness(map_file):
    """Run the real drop method with synchronous threading and a temporary YAML path."""
    source_path = Path(__file__).with_name('ui_node.py')
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'NiceGuiNode'
    )
    method = next(
        node for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name == 'drop_topo_node'
    )
    namespace = {
        'TopoNode': TopoNode,
        'TopoEdge': TopoEdge,
        'TopoProperties': TopoProperties,
        'Vector2': Vector2,
        'NAV_ACTION': NAV_ACTION,
        '_NAME_RE': re.compile(r'^[A-Z0-9_]+$'),
        '_TOPO_SRV_OK': False,
        '_topo_to_msg': lambda doc: doc.to_dict(),
        'copy': copy,
        're': re,
        'traceback': traceback,
        'UTC': UTC,
        'datetime': datetime,
        'default_actions': default_actions,
        'default_definitions': default_definitions,
        'os': SimpleNamespace(path=SimpleNamespace(exists=lambda _: map_file.exists())),
        'parse_topo_yaml': lambda _: parse_topo_yaml(map_file),
        'dump_topo_yaml': Mock(side_effect=lambda doc, _: dump_topo_yaml(doc, map_file)),
        'threading': SimpleNamespace(
            Thread=lambda target, daemon: SimpleNamespace(start=target)),
    }
    exec(compile(ast.Module(body=[method], type_ignores=[]), source_path, 'exec'), namespace)
    return type('DropHarness', (), {'drop_topo_node': namespace['drop_topo_node']}), namespace


class TestDropTopoNode(unittest.TestCase):
    def setUp(self) -> None:
        # NOTE: enterContext owns cleanup.
        # pylint: disable-next=consider-using-with
        temp_dir = self.enterContext(TemporaryDirectory())
        self.map_file = Path(temp_dir) / 'field.yaml'
        harness, self.namespace = load_drop_harness(self.map_file)
        self.node = harness()
        self.node._topo_doc = TopoDoc(name='field.yaml')
        self.node._run_vm = RunViewModel()
        self.node._is_sim = True
        self.node._row_action = ROW_ACTION
        self.node.latest_odom = SimpleNamespace(
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.234, y=5.678))))
        self.node.latest_gps = None
        self.node.get_logger = Mock(return_value=Mock())
        self.node._topo_map_pub = Mock()

    def test_missing_map_saves_node_and_defaults(self) -> None:
        self.node.drop_topo_node('new', 7, 'exit')

        saved = parse_topo_yaml(self.map_file)
        self.assertEqual({node.name for node in saved.nodes}, {'NEW'})
        new_node = saved.get_node('NEW')
        self.assertEqual((new_node.x, new_node.y), (1.234, 5.678))
        self.assertEqual(new_node.meta['row_id'], 7)
        self.assertEqual(new_node.meta['row_role'], 'exit')
        self.assertEqual(list(new_node.edges), [])
        self.assertEqual(saved.actions, default_actions())
        self.assertEqual(saved.definitions, default_definitions())
        self.assertTrue(self.node._topo_doc.has_node('NEW'))
        self.node._topo_map_pub.publish.assert_called_once()

    def test_missing_map_prunes_connection_to_unsaved_target(self) -> None:
        self.node._topo_doc.add_node(TopoNode(name='TARGET', x=0.0, y=0.0))
        self.node._run_vm.topo.current_node = 'TARGET'
        source = self.node._topo_doc

        self.node.drop_topo_node('NEW', None)

        saved = parse_topo_yaml(self.map_file)
        self.assertEqual({node.name for node in saved.nodes}, {'NEW'})
        self.assertEqual(list(saved.get_node('NEW').edges), [])
        self.assertTrue(source.get_node('NEW').is_connected_to('TARGET'))
        self.assertNotIn('ERROR', self.node._run_vm.drop_node.status)

    def test_existing_map_keeps_connection_to_saved_target(self) -> None:
        self.node._topo_doc.add_node(TopoNode(name='TARGET', x=0.0, y=0.0))
        self.node._run_vm.topo.selected_node = 'TARGET'
        dump_topo_yaml(self.node._topo_doc, self.map_file)

        self.node.drop_topo_node('NEW', None)

        saved = parse_topo_yaml(self.map_file)
        self.assertEqual({node.name for node in saved.nodes}, {'TARGET', 'NEW'})
        self.assertEqual(list(saved.get_node('NEW').edges), [
            TopoEdge(action=NAV_ACTION, edge_id='NEW_TARGET', node='TARGET')])

    def test_existing_map_prunes_connection_to_missing_target(self) -> None:
        self.node._topo_doc.add_node(TopoNode(name='TARGET', x=0.0, y=0.0))
        self.node._run_vm.topo.current_node = 'TARGET'
        dump_topo_yaml(TopoDoc(name='field.yaml'), self.map_file)

        self.node.drop_topo_node('NEW', None)

        saved = parse_topo_yaml(self.map_file)
        self.assertTrue(saved.has_node('NEW'))
        self.assertEqual(list(saved.get_node('NEW').edges), [])

    def test_disk_duplicate_is_not_overwritten(self) -> None:
        disk_doc = TopoDoc(name='field.yaml', nodes=[TopoNode(name='NEW', x=9.0, y=8.0)])
        dump_topo_yaml(disk_doc, self.map_file)
        original = self.map_file.read_bytes()

        self.node.drop_topo_node('NEW', None)

        self.assertEqual(self.map_file.read_bytes(), original)
        self.namespace['dump_topo_yaml'].assert_not_called()
        self.node._topo_map_pub.publish.assert_not_called()
