# pylint: disable=protected-access,attribute-defined-outside-init
import copy
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock

from devkit_ui.constants import ROW_ACTION, VISION_ROW_ACTION
from devkit_ui.models import TopoDoc, TopoNode
from devkit_ui.parse import dump_topo_yaml, parse_topo_yaml
from devkit_ui.test_map_operations import load_map_harness
from devkit_ui.topo_defaults import default_actions, default_definitions


class TestMapFailures(unittest.TestCase):
    def setUp(self) -> None:
        """Create a persisted map in vision mode and capture its original state."""
        # pylint: disable-next=consider-using-with
        self.directory = Path(self.enterContext(TemporaryDirectory()))
        self.node, self.namespace = load_map_harness(self.directory, self)
        self.live = self.node._topo_doc
        self.node._row_action = self.node._run_vm.drop_node.row_action = VISION_ROW_ACTION
        dump_topo_yaml(self.live, self.directory / 'live')
        self.original_bytes = (self.directory / 'live').read_bytes()

    def test_archive_read_failure_does_not_clear_or_reset_mode(self) -> None:
        """Verify archive read errors preserve the live map, saved file, and row mode."""
        self.namespace['parse_topo_yaml'].side_effect = ValueError('invalid map')

        self.assertEqual(self.node.archive_and_clear_map(), 'ERROR: invalid map')

        self.namespace['dump_topo_yaml'].assert_not_called()
        self.assertIs(self.node._topo_doc, self.live)
        self.assertEqual(self.node._row_action, VISION_ROW_ACTION)
        self.assertEqual(self.node._run_vm.drop_node.row_action, VISION_ROW_ACTION)
        self.assertEqual((self.directory / 'live').read_bytes(), self.original_bytes)
        self.node._topo_map_pub.publish.assert_not_called()

    def test_clear_write_failure_retains_archive_and_live_state(self) -> None:
        """Verify a failed clear retains the completed archive and the original live state."""
        write = self.namespace['dump_topo_yaml']
        original_write = write.side_effect

        def fail_live_write(doc, path):
            """Allow archive writes but reject writes to the live map file."""
            if Path(path).name == 'live':
                raise OSError('read only')
            original_write(doc, path)

        write.side_effect = fail_live_write

        self.assertEqual(self.node.archive_and_clear_map(), 'ERROR: read only')

        self.assertEqual(write.call_count, 2)
        archived = parse_topo_yaml(self.directory / 'live_1').to_dict()
        persisted = parse_topo_yaml(self.directory / 'live').to_dict()
        # NOTE: serialization refreshes this timestamp when writing the archive.
        archived['meta'].pop('last_updated')
        persisted['meta'].pop('last_updated')
        self.assertEqual(archived, persisted)
        self.assertEqual((self.directory / 'live').read_bytes(), self.original_bytes)
        self.assertIs(self.node._topo_doc, self.live)
        self.assertEqual(self.node._row_action, VISION_ROW_ACTION)
        self.assertEqual(self.node._run_vm.drop_node.row_action, VISION_ROW_ACTION)
        self.node._topo_map_pub.publish.assert_not_called()

    def test_publish_failure_does_not_undo_successful_clear(self) -> None:
        """Verify a publish error leaves a successful archive and map reset intact."""
        self.node._topo_map_pub.publish.side_effect = RuntimeError('publisher unavailable')

        self.assertEqual(self.node.archive_and_clear_map(), 'archived → live_1')

        cleared = parse_topo_yaml(self.directory / 'live')
        self.assertEqual(list(cleared.nodes), [])
        self.assertEqual(cleared.actions, default_actions())
        self.assertEqual(cleared.definitions, default_definitions())
        self.assertEqual(self.node._topo_doc.to_dict(), cleared.to_dict())
        self.assertTrue(parse_topo_yaml(self.directory / 'live_1').has_node('A'))
        self.assertEqual(self.node._row_action, ROW_ACTION)
        self.assertEqual(self.node._run_vm.drop_node.row_action, ROW_ACTION)

    def test_save_directory_failure_preserves_live_map_and_selection(self) -> None:
        """Verify directory creation errors leave the live map and selected mode intact."""
        self.namespace['os'].makedirs.side_effect = OSError('permission denied')

        self.assertEqual(self.node.save_map_as('copy'), 'ERROR: permission denied')

        self.assertFalse((self.directory / 'copy').exists())
        self.assertEqual((self.directory / 'live').read_bytes(), self.original_bytes)
        self.assertIs(self.node._topo_doc, self.live)
        self.assertEqual(self.node._row_action, VISION_ROW_ACTION)
        self.namespace['dump_topo_yaml'].assert_not_called()
        self.node._topo_map_pub.publish.assert_not_called()

    def test_persist_read_failure_does_not_fall_back_to_memory(self) -> None:
        """Verify a disk read error stops modification, saving, and publishing."""
        self.namespace['parse_topo_yaml'].side_effect = ValueError('invalid map')
        modify = Mock()

        self.node._persist_and_reload(modify, self.node._run_vm.drop_node, 'status', 'updated')

        self.assertEqual(self.node._run_vm.drop_node.status, 'ERROR: invalid map')
        modify.assert_not_called()
        self.namespace['dump_topo_yaml'].assert_not_called()
        self.assertIs(self.node._topo_doc, self.live)
        self.assertEqual((self.directory / 'live').read_bytes(), self.original_bytes)
        self.node._topo_map_pub.publish.assert_not_called()

    def test_missing_file_modification_failure_does_not_mutate_live_document(self) -> None:
        """Verify a failing edit to a new map leaves the live document unchanged."""
        (self.directory / 'live').unlink()
        original = copy.deepcopy(self.live.to_dict())

        def modify_then_fail(doc):
            """Add a node to the working copy before simulating an edit failure."""
            doc.insert_node(TopoNode(name='NEW'))
            raise ValueError('invalid modification')

        self.node._persist_and_reload(
            modify_then_fail, self.node._run_vm.drop_node, 'status', 'updated')

        self.assertEqual(self.node._run_vm.drop_node.status, 'ERROR: invalid modification')
        self.assertIs(self.node._topo_doc, self.live)
        self.assertEqual(self.live.to_dict(), original)
        self.assertFalse((self.directory / 'live').exists())
        self.namespace['dump_topo_yaml'].assert_not_called()
        self.node._topo_map_pub.publish.assert_not_called()

    def test_persist_existing_file_keeps_disk_settings_instead_of_seeding_defaults(self) -> None:
        """Verify persistence modifies the saved map while retaining its custom settings."""
        disk = TopoDoc(name='live', nodes=[TopoNode(name='DISK', x=3.0, y=4.0)],
                       actions={'custom': {'composable': False}}, definitions={'bt': '<custom/>'})
        dump_topo_yaml(disk, self.directory / 'live')

        self.node._persist_and_reload(
            lambda doc: doc.insert_node(TopoNode(name='NEW', x=5.0, y=6.0)),
            self.node._run_vm.drop_node, 'status', 'updated')

        saved = parse_topo_yaml(self.directory / 'live')
        self.assertEqual({node.name for node in saved.nodes}, {'DISK', 'NEW'})
        self.assertEqual(saved.actions, disk.actions)
        self.assertEqual(saved.definitions, disk.definitions)
        self.assertEqual(saved.get_node('NEW').meta,
                         {'map': 'live', 'pointset': 'live', 'node': 'NEW'})
        self.assertEqual(self.node._run_vm.drop_node.status, 'updated — live (no srv)')
        self.node._topo_map_pub.publish.assert_called_once_with(self.node._topo_doc.to_dict())
