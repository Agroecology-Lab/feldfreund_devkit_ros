"""Tests for the topology view model, application service and domain service."""
import json
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.application_services.topology_application_service import (
    TopologyApplicationService,
    create_default_doc,
)
from devkit_ui.models import TopoDoc, TopoNode
from devkit_ui.view_models.topology_view_model import TopologyViewModel

# The domain service needs rclpy and std_msgs at import time; stub them.
_rclpy_qos = ModuleType('rclpy.qos')
_rclpy_qos.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL='transient_local')
_rclpy_qos.HistoryPolicy = SimpleNamespace(KEEP_LAST='keep_last')
_rclpy_qos.ReliabilityPolicy = SimpleNamespace(RELIABLE='reliable')
_rclpy_qos.QoSProfile = lambda **_kwargs: object()
_rclpy_node = ModuleType('rclpy.node')
_rclpy_node.Node = type('Node', (), {})
_std_msgs = ModuleType('std_msgs.msg')
_std_msgs.String = type('String', (), {'data': ''})

with patch.dict(sys.modules, {
    'rclpy': ModuleType('rclpy'),
    'rclpy.node': _rclpy_node,
    'rclpy.qos': _rclpy_qos,
    'std_msgs': ModuleType('std_msgs'),
    'std_msgs.msg': _std_msgs,
}):
    from devkit_ui.domain_services.topology_domain_service import TopologyDomainService


def make_stack():
    ros = Mock()
    domain = TopologyDomainService(ros)
    app = TopologyApplicationService(domain)
    vm = TopologyViewModel(app)
    return ros, domain, app, vm


class TestTopologyStack(unittest.TestCase):
    def test_default_doc_has_linear_chain(self) -> None:
        """Verify the demo map is a six-node chain."""
        doc = create_default_doc()
        self.assertEqual([n.name for n in doc.nodes], ['N1', 'N2', 'N3', 'N4', 'N5', 'N6'])

    def test_set_doc_reaches_view_model_and_clears_stale_selection(self) -> None:
        """Verify replacing the doc mirrors it and drops a selection that no longer exists."""
        _, _, app, vm = make_stack()
        app.set_doc(TopoDoc(name='a', nodes=[TopoNode(name='X'), TopoNode(name='Y')]))
        vm.set_selected_node('X')
        app.set_doc(TopoDoc(name='a', nodes=[TopoNode(name='Y')]))

        self.assertEqual([n.name for n in vm.topo_doc.nodes], ['Y'])
        self.assertIsNone(vm.selected_node)

    def test_set_doc_keeps_selection_that_still_exists(self) -> None:
        """Verify a surviving node stays selected across a doc replacement."""
        _, _, app, vm = make_stack()
        app.set_doc(TopoDoc(name='a', nodes=[TopoNode(name='X')]))
        vm.set_selected_node('X')
        app.set_doc(TopoDoc(name='a', nodes=[TopoNode(name='X'), TopoNode(name='Y')]))

        self.assertEqual(vm.selected_node, 'X')

    def test_publish_sends_current_doc_synchronously(self) -> None:
        """Verify publish() emits the live doc as JSON on the topic publisher."""
        ros, _, app, _ = make_stack()
        publisher = ros.create_publisher.return_value
        app.set_doc(TopoDoc(name='a', nodes=[TopoNode(name='X')]))

        app.publish()

        publisher.publish.assert_called_once()
        self.assertEqual(json.loads(publisher.publish.call_args.args[0].data)['name'], 'a')

    def test_publish_without_doc_is_a_no_op(self) -> None:
        """Verify nothing is published before a doc exists."""
        ros, _, app, _ = make_stack()

        app.publish()

        ros.create_publisher.return_value.publish.assert_not_called()

    def test_publish_failure_propagates_to_caller(self) -> None:
        """Verify callers see publisher errors rather than losing them in a thread."""
        ros, _, app, _ = make_stack()
        app.set_doc(TopoDoc(name='a'))
        ros.create_publisher.return_value.publish.side_effect = RuntimeError('down')

        with self.assertRaises(RuntimeError):
            app.publish()

    def test_add_node_rejects_duplicates_and_missing_doc(self) -> None:
        """Verify add_node validates before mutating or publishing."""
        ros, _, app, _ = make_stack()
        with self.assertRaises(ValueError):
            app.add_node(TopoNode(name='X'))
        app.set_doc(TopoDoc(name='a', nodes=[TopoNode(name='X')]))
        with self.assertRaises(ValueError):
            app.add_node(TopoNode(name='X'))
        ros.create_publisher.return_value.publish.assert_not_called()

    def test_incoming_map_replaces_doc(self) -> None:
        """Verify a message on the map topic updates the view model."""
        ros, _, _, vm = make_stack()
        callback = ros.create_subscription.call_args.args[2]
        payload = TopoDoc(name='remote', nodes=[TopoNode(name='R', x=1.0, y=2.0)]).to_dict()

        callback(SimpleNamespace(data=json.dumps(payload)))

        self.assertEqual(vm.topo_doc.name, 'remote')

    def test_default_doc_contains_spaced_nodes_and_bidirectional_edges(self) -> None:
        doc = create_default_doc()
        self.assertEqual(doc.name, 'mixed_test_map')
        self.assertEqual([(node.x, node.y) for node in doc.nodes],
                         [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0),
                          (9.0, 0.0), (12.0, 0.0), (15.0, 0.0)])
        self.assertEqual([[edge.node for edge in node.edges] for node in doc.nodes],
                         [['N2'], ['N1', 'N3'], ['N2', 'N4'],
                          ['N3', 'N5'], ['N4', 'N6'], ['N5']])

    def test_default_initialization_syncs_vm_without_publishing_or_sharing_state(self) -> None:
        ros, domain, app, vm = make_stack()
        app.initialize_with_default()
        first = app.get_doc()
        self.assertIs(first, domain.get_doc())
        self.assertIs(first, vm.topo_doc)
        first.remove_node('N1')

        app.initialize_with_default()

        self.assertIsNot(app.get_doc(), first)
        self.assertIs(vm.topo_doc, app.get_doc())
        self.assertTrue(vm.has_node('N1'))
        ros.create_publisher.return_value.publish.assert_not_called()

    def test_queries_cover_missing_doc_and_existing_node(self) -> None:
        _, _, app, vm = make_stack()
        self.assertIsNone(app.get_doc())
        self.assertFalse(vm.has_node('X'))
        self.assertIsNone(vm.get_node('X'))
        node = TopoNode(name='X', x=2.0, y=-3.0)
        app.set_doc(TopoDoc(name='field', nodes=[node]))

        self.assertTrue(vm.has_node('X'))
        self.assertIs(vm.get_node('X'), node)
        self.assertFalse(vm.has_node('missing'))

    def test_get_missing_node_returns_none_as_documented(self) -> None:
        _, _, app, vm = make_stack()
        app.set_doc(TopoDoc(name='field', nodes=[TopoNode(name='X')]))

        self.assertIsNone(vm.get_node('missing'))

    def test_clear_doc_clears_selection_without_publishing(self) -> None:
        ros, _, app, vm = make_stack()
        app.initialize_with_default()
        vm.set_selected_node('N1')

        app.set_doc(None)
        app.publish()

        self.assertIsNone(app.get_doc())
        self.assertIsNone(vm.topo_doc)
        self.assertIsNone(vm.selected_node)
        ros.create_publisher.return_value.publish.assert_not_called()

    def test_selection_and_localization_are_independent(self) -> None:
        ros, _, app, vm = make_stack()
        app.initialize_with_default()
        vm.set_selected_node('N2')
        vm.set_current_node('N1')
        self.assertEqual(vm.selected_node, 'N2')
        vm.set_selected_node(None)
        self.assertEqual(vm.current_node, 'N1')
        self.assertIsNone(vm.selected_node)
        ros.create_publisher.return_value.publish.assert_not_called()

    def test_add_then_remove_publish_distinct_snapshots_in_order(self) -> None:
        ros, _, app, vm = make_stack()
        app.set_doc(TopoDoc(name='field', nodes=[TopoNode(name='A')]))
        added = TopoNode(name='B', x=2.0, y=3.0, edges=['A'])

        vm.add_node(added)
        self.assertIs(vm.get_node('B'), added)
        vm.remove_node('B')

        messages = [entry.args[0] for entry in
                    ros.create_publisher.return_value.publish.call_args_list]
        self.assertEqual(len(messages), 2)
        self.assertIsNot(messages[0], messages[1])
        added_payload, removed_payload = [json.loads(msg.data) for msg in messages]
        self.assertEqual([entry['node']['name'] for entry in added_payload['nodes']], ['A', 'B'])
        self.assertEqual(added_payload['nodes'][0]['node']['edges'][0]['node'], 'B')
        self.assertEqual(removed_payload, app.get_doc().to_dict())
        self.assertEqual([entry['node']['name'] for entry in removed_payload['nodes']], ['A'])
        self.assertEqual(removed_payload['nodes'][0]['node']['edges'], [])

    def test_remove_rejects_unloaded_or_missing_node_without_side_effects(self) -> None:
        ros, _, app, vm = make_stack()
        with self.assertRaisesRegex(ValueError, 'No topology document loaded'):
            vm.remove_node('missing')
        app.set_doc(TopoDoc(name='field', nodes=[TopoNode(name='A')]))
        before = app.get_doc().to_dict()
        with self.assertRaisesRegex(ValueError, 'not found'):
            vm.remove_node('missing')
        self.assertEqual(app.get_doc().to_dict(), before)
        ros.create_publisher.return_value.publish.assert_not_called()

    def test_invalid_incoming_maps_preserve_doc_and_selection_then_recover(self) -> None:
        ros, _, app, vm = make_stack()
        original = TopoDoc(name='field', nodes=[TopoNode(name='A')])
        app.set_doc(original)
        vm.set_selected_node('A')
        callback = ros.create_subscription.call_args.args[2]

        for payload in ('{', 'null', '[]', '{"nodes": [{"node": {}}]}'):
            with self.subTest(payload=payload):
                ros.get_logger.return_value.reset_mock()
                callback(SimpleNamespace(data=payload))
                self.assertIs(app.get_doc(), original)
                self.assertIs(vm.topo_doc, original)
                self.assertEqual(vm.selected_node, 'A')
                ros.get_logger.return_value.error.assert_called_once()
                self.assertIn('/topological_map_2', ros.get_logger.return_value.error.call_args.args[0])

        replacement = TopoDoc(name='recovered', nodes=[TopoNode(name='B', x=1.0, y=2.0)])
        callback(SimpleNamespace(data=json.dumps(replacement.to_dict())))
        self.assertEqual(vm.topo_doc.to_dict(), replacement.to_dict())
        self.assertIsNone(vm.selected_node)
        ros.create_publisher.return_value.publish.assert_not_called()

    def test_subscriber_echo_clears_selection_after_local_removal(self) -> None:
        ros, _, app, vm = make_stack()
        app.set_doc(TopoDoc(name='field', nodes=[TopoNode(name='A')]))
        vm.set_selected_node('A')
        vm.remove_node('A')
        message = ros.create_publisher.return_value.publish.call_args.args[0]

        ros.create_subscription.call_args.args[2](message)

        self.assertIsNone(vm.selected_node)
        self.assertEqual(list(vm.topo_doc.nodes), [])

    def test_publish_preserves_unicode_and_complete_document(self) -> None:
        ros, _, app, _ = make_stack()
        doc = TopoDoc(name='área', metric_map='field', nodes=[
            TopoNode(name='árvore', x=-1.25, y=2.5, meta={'row_id': 2, 'row_role': 'entry'}),
        ])
        app.set_doc(doc)

        app.publish()

        message = ros.create_publisher.return_value.publish.call_args.args[0]
        self.assertIn('árvore', message.data)
        self.assertEqual(json.loads(message.data), doc.to_dict())


if __name__ == '__main__':
    unittest.main()
