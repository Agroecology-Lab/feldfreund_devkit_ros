# pylint: disable=protected-access,attribute-defined-outside-init
import copy
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from devkit_ui.constants import NAV_ACTION, ROW_ACTION, VISION_ROW_ACTION
from devkit_ui.models import TopoDoc, TopoEdge, TopoNode
from devkit_ui.parse import dump_topo_yaml, parse_topo_yaml
from devkit_ui.test_map_operations import load_map_harness


class TestRowActions(unittest.TestCase):
    def setUp(self) -> None:
        """Create a map harness with mixed row actions and a headland navigation edge."""
        # pylint: disable-next=consider-using-with
        self.directory = Path(self.enterContext(TemporaryDirectory()))
        self.node, self.namespace = load_map_harness(self.directory)
        self.node._topo_doc = TopoDoc(name='live', nodes=[
            TopoNode(name='A', x=0.0, y=0.0, edges=[
                TopoEdge(ROW_ACTION, 'row', 'B'), TopoEdge(NAV_ACTION, 'headland', 'C')]),
            TopoNode(name='B', x=0.0, y=10.0, edges=[TopoEdge(VISION_ROW_ACTION, 'back', 'A')]),
            TopoNode(name='C', x=2.0, y=0.0),
        ])

    def test_switch_updates_state_and_persists_only_row_edges_in_both_directions(self) -> None:
        """Verify mode changes update session state and save only changes to row edges."""
        for action, count in ((VISION_ROW_ACTION, 1), (ROW_ACTION, 2)):
            with self.subTest(action=action):
                self.node.set_row_action(action)

                self.assertEqual(self.node._row_action, action)
                self.assertEqual(self.node._run_vm.drop_node.row_action, action)
                saved = parse_topo_yaml(self.directory / 'live')
                self.assertEqual(list(saved.get_node('A').edges), [
                    TopoEdge(action, 'row', 'B'), TopoEdge(NAV_ACTION, 'headland', 'C')])
                self.assertEqual(list(saved.get_node('B').edges), [TopoEdge(action, 'back', 'A')])
                self.assertIn(f'{count} row edges updated', self.node._run_vm.drop_node.status)
        self.assertEqual(self.node._topo_map_pub.publish.call_count, 2)

    def test_invalid_or_unchanged_mode_has_no_side_effects(self) -> None:
        """Verify unsupported and unchanged modes leave state and persistence untouched."""
        original = copy.deepcopy(self.node._topo_doc.to_dict())
        self.node._run_vm.drop_node.status = 'previous status'
        for action in ('', 'unsupported', NAV_ACTION, ROW_ACTION):
            with self.subTest(action=action):
                self.node.set_row_action(action)
                self.assertEqual(self.node._row_action, ROW_ACTION)
                self.assertEqual(self.node._run_vm.drop_node.row_action, ROW_ACTION)
                self.assertEqual(self.node._topo_doc.to_dict(), original)
                self.assertEqual(self.node._run_vm.drop_node.status, 'previous status')
        self.namespace['dump_topo_yaml'].assert_not_called()
        self.node._topo_map_pub.publish.assert_not_called()

    def test_mode_can_change_without_map_or_matching_edges(self) -> None:
        """Verify the session mode can change without a map or any row edges to save."""
        for doc in (None, TopoDoc(name='live'), TopoDoc(name='live', nodes=[
            TopoNode(name='A', edges=[TopoEdge(NAV_ACTION, 'headland', 'B')]),
        ])):
            with self.subTest(doc=doc):
                self.node._topo_doc = doc
                self.node._row_action = ROW_ACTION
                self.node._run_vm.drop_node.row_action = ROW_ACTION
                self.node.set_row_action(VISION_ROW_ACTION)
                self.assertEqual(self.node._row_action, VISION_ROW_ACTION)
                self.assertEqual(self.node._run_vm.drop_node.row_action, VISION_ROW_ACTION)
                if doc is not None:
                    self.assertIn('no edges to change', self.node._run_vm.drop_node.status)
        self.namespace['dump_topo_yaml'].assert_not_called()

    def test_switch_edits_persisted_document_and_preserves_disk_only_nodes(self) -> None:
        """Verify switching updates row edges on nodes found only in the saved map."""
        disk = copy.deepcopy(self.node._topo_doc)
        disk.insert_node(TopoNode(name='DISK', x=4.0, y=5.0,
                                 edges=[TopoEdge(ROW_ACTION, 'extra', 'A')]))
        dump_topo_yaml(disk, self.directory / 'live')

        self.node.set_row_action(VISION_ROW_ACTION)

        saved = parse_topo_yaml(self.directory / 'live')
        self.assertEqual(list(saved.get_node('DISK').edges), [
            TopoEdge(VISION_ROW_ACTION, 'extra', 'A')])

    def test_failed_save_keeps_selected_mode_but_preserves_live_and_disk_edges(self) -> None:
        """Verify save failures retain the selected mode and leave map edges unchanged."""
        live = self.node._topo_doc
        dump_topo_yaml(live, self.directory / 'live')
        original = copy.deepcopy(live.to_dict())
        original_bytes = (self.directory / 'live').read_bytes()
        self.namespace['dump_topo_yaml'].side_effect = OSError('read only')

        self.node.set_row_action(VISION_ROW_ACTION)

        self.assertEqual(self.node._run_vm.drop_node.status, 'ERROR: read only')
        self.assertEqual(self.node._row_action, VISION_ROW_ACTION)
        self.assertEqual(self.node._run_vm.drop_node.row_action, VISION_ROW_ACTION)
        self.assertIs(self.node._topo_doc, live)
        self.assertEqual(live.to_dict(), original)
        self.assertEqual((self.directory / 'live').read_bytes(), original_bytes)
        self.node._topo_map_pub.publish.assert_not_called()
