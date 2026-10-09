"""Tests for map persistence in TopoMapStore and TopologyApplicationService."""
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.application_services.topology_application_service import (
    TopologyApplicationService,
)
from devkit_ui.domain_services.topo_map_store import TopoMapStore
from devkit_ui.models import TopoDoc, TopoEdge, TopoNode
from devkit_ui.topo_results import PersistResult, SwitchResult


class FakeDomain:
    """Minimal domain service: holds a doc and records publish/switch calls."""

    def __init__(self, doc=None, switch=None) -> None:
        self.doc = doc
        self.switch = switch or SwitchResult(available=True, success=True)
        self.calls = []

    def get_doc(self):
        return self.doc

    def set_doc(self, doc) -> None:
        self.doc = doc
        self.calls.append('set_doc')

    def publish(self) -> None:
        self.calls.append('publish')

    def switch_map(self, path, timeout=5.0):
        self.calls.append(('switch', path))
        return self.switch


def live_doc() -> TopoDoc:
    return TopoDoc(name='live', nodes=[TopoNode(name='A', x=1.0, y=2.0)])


class ServiceCase(unittest.TestCase):
    def make(self, doc=None, switch=None):
        directory = Path(self.enterContext(TemporaryDirectory()))
        self.store = TopoMapStore(str(directory))
        self.domain = FakeDomain(doc if doc is not None else live_doc(), switch)
        self.app = TopologyApplicationService(self.domain, self.store)
        return directory


class TestTopoMapStore(unittest.TestCase):
    def test_roundtrip_creates_directory_and_names_archives(self) -> None:
        """Verify save creates the directory, load reads it back, and archives count up."""
        directory = Path(self.enterContext(TemporaryDirectory())) / 'maps'
        store = TopoMapStore(str(directory))
        store.save(live_doc())

        self.assertTrue(store.exists('live'))
        self.assertEqual([n.name for n in store.load('live').nodes], ['A'])
        self.assertEqual(store.next_archive_name('live'), 'live_1')
        store.save(live_doc(), 'live_1')
        self.assertEqual(store.next_archive_name('live'), 'live_2')


class TestPersistAndReload(ServiceCase):
    def test_switch_success_is_live_and_does_not_publish(self) -> None:
        """Verify a successful switch needs no republish."""
        self.make()

        result = self.app.persist_and_reload(lambda d: d.insert_node(TopoNode(name='B', x=0.0, y=0.0)))

        self.assertEqual(result, PersistResult('live'))
        self.assertNotIn('publish', self.domain.calls)
        self.assertEqual({n.name for n in self.store.load('live').nodes}, {'A', 'B'})
        self.assertEqual({n.name for n in self.domain.doc.nodes}, {'A', 'B'})
        self.assertEqual(self.domain.calls[-1], ('switch', self.store.path('live')))

    def test_switch_failure_and_missing_service_publish_instead(self) -> None:
        """Verify both fallback outcomes publish and report their reason."""
        cases = ((SwitchResult(available=True, success=False, message='timeout'),
                  PersistResult('switch_failed', 'timeout')),
                 (SwitchResult(available=False), PersistResult('no_srv')))
        for switch, expected in cases:
            with self.subTest(expected=expected.kind):
                self.make(switch=switch)
                result = self.app.persist_and_reload(lambda d: None)
                self.assertEqual(result, expected)
                self.assertEqual(self.domain.calls.count('publish'), 1)

    def test_modify_error_leaves_live_doc_and_file_untouched(self) -> None:
        """Verify an exception in modify writes nothing and does not replace the live doc."""
        self.make()
        live = self.domain.doc

        def boom(_doc):
            raise RuntimeError('nope')

        with self.assertRaises(RuntimeError):
            self.app.persist_and_reload(boom)

        self.assertIs(self.domain.doc, live)
        self.assertFalse(self.store.exists('live'))
        self.assertEqual(self.domain.calls, [])

    def test_no_document_raises(self) -> None:
        """Verify persisting without a loaded map fails clearly."""
        self.make()
        self.domain.doc = None
        with self.assertRaisesRegex(ValueError, 'map not loaded'):
            self.app.persist_and_reload(lambda d: None)

    def test_describe_texts(self) -> None:
        """Verify status wording for each outcome."""
        self.assertEqual(PersistResult('live').describe('done'), 'done — live')
        self.assertEqual(PersistResult('no_srv').describe('done'), 'done — live (no srv)')
        self.assertEqual(PersistResult('switch_failed', 'x').describe('done'),
                         'done (switch failed: x)')


class TestDropNode(ServiceCase):
    def test_saved_before_on_saved_before_switch(self) -> None:
        """Verify on_saved fires after the live doc is replaced and before the switch."""
        self.make()
        order = []
        self.domain.calls = order

        result = self.app.drop_node(TopoNode(name='B', x=0.0, y=0.0),
                                    on_saved=lambda: order.append('on_saved'))

        self.assertEqual(result.kind, 'live')
        self.assertEqual(order[:2], ['set_doc', 'on_saved'])
        self.assertEqual(order[2][0], 'switch')

    def test_duplicate_on_disk_is_skipped(self) -> None:
        """Verify a name already on disk writes nothing and does not switch."""
        self.make()
        self.store.save(live_doc())
        before = Path(self.store.path('live')).read_bytes()

        result = self.app.drop_node(TopoNode(name='A', x=5.0, y=5.0))

        self.assertEqual(result.kind, 'skipped')
        self.assertEqual(Path(self.store.path('live')).read_bytes(), before)
        self.assertEqual(self.domain.calls, [])

    def test_edges_to_unsaved_nodes_are_pruned_from_file_only(self) -> None:
        """Verify the saved copy drops edges to unsaved targets without altering the argument."""
        self.make()
        self.store.save(live_doc())
        node = TopoNode(name='B', x=0.0, y=0.0, edges=[
            TopoEdge('navigate_to_pose', 'B_A', 'A'), TopoEdge('navigate_to_pose', 'B_Z', 'Z')])

        self.app.drop_node(node)

        saved = self.store.load('live').get_node('B')
        self.assertEqual([e.node for e in saved.edges], ['A'])
        self.assertEqual(len(node.edges), 2)


class TestPatchNodeRole(ServiceCase):
    def test_no_file_is_a_no_op(self) -> None:
        """Verify nothing is written when the map has no file yet."""
        self.make()
        self.app.patch_node_role('A', 'exit')
        self.assertFalse(self.store.exists('live'))


class TestDomainSwitchMap(unittest.TestCase):
    """TopologyDomainService.switch_map with the ROS service client faked."""

    def setUp(self) -> None:
        rclpy_qos = ModuleType('rclpy.qos')
        rclpy_qos.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1)
        rclpy_qos.HistoryPolicy = SimpleNamespace(KEEP_LAST=1)
        rclpy_qos.ReliabilityPolicy = SimpleNamespace(RELIABLE=1)
        rclpy_qos.QoSProfile = lambda **_kw: object()
        rclpy_node = ModuleType('rclpy.node')
        rclpy_node.Node = type('Node', (), {})
        std_msgs = ModuleType('std_msgs.msg')
        std_msgs.String = type('String', (), {'data': ''})
        srv = ModuleType('topological_navigation_msgs.srv')
        srv.WriteTopologicalMap = SimpleNamespace(
            Request=lambda: SimpleNamespace(filename='', no_alias=False))
        stubs = {'rclpy': ModuleType('rclpy'), 'rclpy.qos': rclpy_qos, 'rclpy.node': rclpy_node,
                 'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': std_msgs,
                 'topological_navigation_msgs': ModuleType('topological_navigation_msgs'),
                 'topological_navigation_msgs.srv': srv}
        self.enterContext(patch.dict(sys.modules, stubs))
        sys.modules.pop('devkit_ui.domain_services.topology_domain_service', None)
        from devkit_ui.domain_services import topology_domain_service as module
        self.module = module
        self.enterContext(patch.object(module, 'WriteTopologicalMap', srv.WriteTopologicalMap))
        self.addCleanup(sys.modules.pop, 'devkit_ui.domain_services.topology_domain_service', None)

    def service(self, response):
        """Build the service with a client that answers immediately with response."""
        ros = Mock()
        client = ros.create_client.return_value

        def call_async(request):
            self.request = request
            future = Mock()
            future.result.return_value = response
            future.add_done_callback.side_effect = lambda cb: cb(future)
            return future

        client.call_async.side_effect = call_async
        return self.module.TopologyDomainService(ros)

    def test_success_and_failure_responses(self) -> None:
        """Verify the request carries the path and responses map to SwitchResult."""
        ok = self.service(SimpleNamespace(success=True, message=''))
        self.assertEqual(ok.switch_map('/m/field'), SwitchResult(True, True, ''))
        self.assertEqual((self.request.filename, self.request.no_alias), ('/m/field', True))

        bad = self.service(SimpleNamespace(success=False, message='bad map'))
        self.assertEqual(bad.switch_map('/m/field'), SwitchResult(True, False, 'bad map'))

    def test_timeout_reports_unsuccessful(self) -> None:
        """Verify a response that never arrives reports a timeout."""
        ros = Mock()
        ros.create_client.return_value.call_async.return_value = Mock()  # never calls back
        domain = self.module.TopologyDomainService(ros)

        self.assertEqual(domain.switch_map('/m/field', timeout=0.01),
                         SwitchResult(True, False, 'timeout'))

    def test_without_message_package_service_is_unavailable(self) -> None:
        """Verify missing topological_navigation_msgs gives available=False."""
        with patch.object(self.module, 'WriteTopologicalMap', None):
            domain = self.module.TopologyDomainService(Mock())
        self.assertEqual(domain.switch_map('/m/field'), SwitchResult(available=False))


if __name__ == '__main__':
    unittest.main()
