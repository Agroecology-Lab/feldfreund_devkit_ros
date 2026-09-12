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


if __name__ == '__main__':
    unittest.main()
