# pylint: disable=exec-used,no-member,protected-access,attribute-defined-outside-init
import ast
import copy
import os
import re
import traceback
import unittest
from collections.abc import Callable
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock

from devkit_ui.constants import ROW_ACTION, VISION_ROW_ACTION
from devkit_ui.models import TopoDoc, TopoNode
from devkit_ui.parse import dump_topo_yaml, parse_topo_yaml
from devkit_ui.topo_defaults import default_actions, default_definitions
from devkit_ui.view_models.run_view_model import RunViewModel


def load_map_harness(directory):
    """Load real map methods, redirecting filesystem boundaries to a temporary directory."""
    source_path = Path(__file__).with_name('ui_node.py')
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    node_class = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == 'NiceGuiNode')
    names = {'save_map_as', 'archive_and_clear_map', '_persist_and_reload', 'set_row_action'}
    members = [node for node in node_class.body
               if (isinstance(node, ast.FunctionDef) and node.name in names)
               or (isinstance(node, ast.Assign)
               and any(isinstance(target, ast.Name) and target.id == '_MAP_NAME_RE'
                       for target in node.targets))]
    harness = ast.ClassDef(name='MapHarness', bases=[], keywords=[], body=members,
                          decorator_list=[])

    def local_path(path):
        return directory / Path(path).relative_to('/workspace/maps')

    namespace = {
        'copy': copy, 're': re, 'traceback': traceback, 'Callable': Callable, 'TopoDoc': TopoDoc,
        'ROW_ACTION': ROW_ACTION, 'VISION_ROW_ACTION': VISION_ROW_ACTION,
        'default_actions': default_actions, 'default_definitions': default_definitions,
        '_TOPO_SRV_OK': False, '_topo_to_msg': lambda doc: doc.to_dict(),
        'os': SimpleNamespace(
            path=SimpleNamespace(exists=lambda path: local_path(path).exists(),
                                 basename=os.path.basename),
            makedirs=Mock(side_effect=lambda path, **kw: local_path(path).mkdir(**kw)),
        ),
        'parse_topo_yaml': Mock(side_effect=lambda path: parse_topo_yaml(local_path(path))),
        'dump_topo_yaml': Mock(side_effect=lambda doc, path: dump_topo_yaml(doc, local_path(path))),
        'threading': SimpleNamespace(
            Thread=lambda target, daemon: SimpleNamespace(start=target)),
    }
    module = ast.fix_missing_locations(ast.Module(body=[harness], type_ignores=[]))
    exec(compile(module, source_path, 'exec'), namespace)
    node = namespace['MapHarness']()
    node._topo_doc = TopoDoc(name='live', nodes=[TopoNode(name='A', x=1.0, y=2.0)])
    node._run_vm = RunViewModel()
    node._row_action = ROW_ACTION
    node._topo_map_pub = Mock()
    node.get_logger = Mock(return_value=Mock())
    return node, namespace


class TestMapOperations(unittest.TestCase):
    def setUp(self) -> None:
        # pylint: disable-next=consider-using-with
        self.directory = Path(self.enterContext(TemporaryDirectory()))
        self.node, self.namespace = load_map_harness(self.directory)

    def test_save_prefers_persisted_state_and_leaves_live_map_untouched(self) -> None:
        persisted = TopoDoc(name='live', metric_map='occupancy',
                            nodes=[TopoNode(name='DISK', x=9.0, y=8.0)])
        dump_topo_yaml(persisted, self.directory / 'live')
        original_bytes = (self.directory / 'live').read_bytes()
        live = self.node._topo_doc
        original = copy.deepcopy(live.to_dict())

        self.assertEqual(self.node.save_map_as('  north_field-2026  '),
                         'saved → north_field-2026')

        saved = parse_topo_yaml(self.directory / 'north_field-2026')
        self.assertEqual(saved.name, 'north_field-2026')
        self.assertEqual({node.name for node in saved.nodes}, {'DISK'})
        self.assertEqual(saved.get_node('DISK').meta['map'], 'north_field-2026')
        self.assertEqual(saved.to_dict()['metric_map'], 'occupancy')
        self.assertEqual((self.directory / 'live').read_bytes(), original_bytes)
        self.assertIs(self.node._topo_doc, live)
        self.assertEqual(live.to_dict(), original)
        self.node._topo_map_pub.publish.assert_not_called()

    def test_save_without_disk_source_accepts_name_boundaries(self) -> None:
        for name in ('7', 'a' * 64):
            with self.subTest(name=name):
                self.assertEqual(self.node.save_map_as(name), f'saved → {name}')
                saved = parse_topo_yaml(self.directory / name)
                self.assertTrue(saved.has_node('A'))
                self.assertEqual(saved.name, name)
        self.assertFalse((self.directory / 'live').exists())
        self.assertEqual(self.node._topo_doc.name, 'live')

    def test_invalid_names_never_touch_filesystem(self) -> None:
        for name in (None, '', ' ', '../escape', '/absolute', 'a/b', 'a\\b', '.hidden',
                     '_field', '-field', 'two words', 'field.yaml', 'é', 'a' * 65, 'a\nb'):
            with self.subTest(name=name):
                self.assertTrue(self.node.save_map_as(name).startswith('ERROR: use letters'))
        self.namespace['parse_topo_yaml'].assert_not_called()
        self.namespace['dump_topo_yaml'].assert_not_called()
        self.namespace['os'].makedirs.assert_not_called()
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_save_refuses_live_name_and_existing_target(self) -> None:
        target = self.directory / 'existing'
        target.write_text('keep me', encoding='utf-8')
        self.assertEqual(self.node.save_map_as(' live '), 'ERROR: that is the live map name')
        self.assertEqual(self.node.save_map_as('existing'), 'ERROR: existing already exists')
        self.assertEqual(target.read_text(encoding='utf-8'), 'keep me')
        self.namespace['dump_topo_yaml'].assert_not_called()

    def test_save_rejects_unloaded_and_empty_maps(self) -> None:
        self.node._topo_doc = None
        self.assertEqual(self.node.save_map_as('copy'), 'ERROR: map not loaded')
        self.node._topo_doc = TopoDoc(name='live')
        self.assertEqual(self.node.save_map_as('copy'), 'ERROR: map has no nodes')
        self.node._topo_doc.add_node(TopoNode(name='MEMORY'))
        dump_topo_yaml(TopoDoc(name='live'), self.directory / 'live')
        self.assertEqual(self.node.save_map_as('copy'), 'ERROR: map has no nodes')
        self.namespace['dump_topo_yaml'].assert_not_called()

    def test_save_reports_read_and_write_errors_without_changing_live_map(self) -> None:
        dump_topo_yaml(self.node._topo_doc, self.directory / 'live')
        live = self.node._topo_doc
        for boundary in ('parse_topo_yaml', 'dump_topo_yaml'):
            with self.subTest(boundary=boundary):
                operation = self.namespace[boundary]
                original_effect = operation.side_effect
                operation.side_effect = OSError('disk unavailable')
                try:
                    self.assertEqual(self.node.save_map_as('copy'), 'ERROR: disk unavailable')
                    self.assertIs(self.node._topo_doc, live)
                    self.assertFalse((self.directory / 'copy').exists())
                finally:
                    operation.side_effect = original_effect
        self.assertEqual(self.node.get_logger().error.call_count, 2)
        self.node._topo_map_pub.publish.assert_not_called()

    def test_clear_archives_custom_settings_then_resets_defaults_and_row_mode(self) -> None:
        live = self.node._topo_doc
        live.seed_actions({'legacy': {'composable': True}}, {'legacy_bt': '<legacy/>'})
        live.transformation['translation'] = {'x': 42.0}
        original = copy.deepcopy(live.to_dict())
        self.node._row_action = VISION_ROW_ACTION
        self.node._run_vm.drop_node.row_action = VISION_ROW_ACTION

        self.assertEqual(self.node.archive_and_clear_map(), 'archived → live_1')

        archived = parse_topo_yaml(self.directory / 'live_1')
        self.assertTrue(archived.has_node('A'))
        self.assertEqual(archived.actions, original['actions'])
        self.assertEqual(archived.definitions, original['definitions'])
        cleared = parse_topo_yaml(self.directory / 'live')
        self.assertEqual(list(cleared.nodes), [])
        self.assertEqual(cleared.actions, default_actions())
        self.assertEqual(cleared.definitions, default_definitions())
        self.assertEqual(cleared.transformation, original['transformation'])
        self.assertEqual(self.node._row_action, ROW_ACTION)
        self.assertEqual(self.node._run_vm.drop_node.row_action, ROW_ACTION)
        self.assertEqual(live.to_dict(), original)
        self.node._topo_map_pub.publish.assert_called_once_with(self.node._topo_doc.to_dict())

    def test_clear_archives_disk_state_without_overwriting_previous_archive(self) -> None:
        dump_topo_yaml(TopoDoc(name='live', nodes=[TopoNode(name='DISK', x=0.0, y=0.0)]),
                       self.directory / 'live')
        previous = self.directory / 'live_1'
        previous.write_text('previous archive', encoding='utf-8')

        self.assertEqual(self.node.archive_and_clear_map(), 'archived → live_2')

        self.assertTrue(parse_topo_yaml(self.directory / 'live_2').has_node('DISK'))
        self.assertEqual(previous.read_text(encoding='utf-8'), 'previous archive')

    def test_failed_clear_keeps_live_document_and_row_mode(self) -> None:
        live = self.node._topo_doc
        self.node._row_action = self.node._run_vm.drop_node.row_action = VISION_ROW_ACTION
        self.namespace['dump_topo_yaml'].side_effect = OSError('read only')

        self.assertEqual(self.node.archive_and_clear_map(), 'ERROR: read only')

        self.assertIs(self.node._topo_doc, live)
        self.assertEqual(self.node._row_action, VISION_ROW_ACTION)
        self.assertEqual(self.node._run_vm.drop_node.row_action, VISION_ROW_ACTION)
        self.node._topo_map_pub.publish.assert_not_called()

    def test_persist_seeds_missing_actions_and_preserves_custom_actions(self) -> None:
        for custom in (False, True):
            with self.subTest(custom=custom):
                node, _ = load_map_harness(self.directory)
                if custom:
                    node._topo_doc.seed_actions({'custom': {'composable': False}},
                                               {'custom_bt': '<custom/>'})
                original = copy.deepcopy(node._topo_doc.to_dict())
                actions = original['actions'] if custom else default_actions()
                definitions = original['definitions'] if custom else default_definitions()
                old_doc = node._topo_doc
                node._persist_and_reload(
                    lambda doc: doc.insert_node(TopoNode(name='NEW', x=0.0, y=0.0)),
                    node._run_vm.drop_node, 'status', 'updated')
                saved = parse_topo_yaml(self.directory / 'live')
                self.assertEqual(saved.actions, actions)
                self.assertEqual(saved.definitions, definitions)
                self.assertEqual({entry.name for entry in saved.nodes}, {'A', 'NEW'})
                self.assertEqual(old_doc.to_dict(), original)
                self.assertIn('updated', node._run_vm.drop_node.status)
                node._topo_map_pub.publish.assert_called_once()
                (self.directory / 'live').unlink()
