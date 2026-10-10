"""Test doubles that wire the topology stack onto a harness node.

The harnesses exec single NiceGuiNode methods in isolation, so they have no ROS node.
``attach_topology`` gives them a real TopologyViewModel and TopologyApplicationService
(with a real TopoMapStore on a temporary directory) on top of a fake domain service.
Publishing lands on ``node._topo_map_pub``, the mock the tests assert on.
"""
import os
from types import SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.application_services.topology_application_service import (
    TopologyApplicationService,
)
from devkit_ui.domain_services import topo_map_store
from devkit_ui.domain_services.topo_map_store import TopoMapStore
from devkit_ui.topo_results import SwitchResult
from devkit_ui.view_models.topology_view_model import TopologyViewModel


class FakeTopologyDomain:
    """Domain-service stand-in that keeps the document on the harness node."""

    def __init__(self, node) -> None:
        """Keep topology on the harness node and default map switching to unavailable."""
        self._state = vars(node)
        self._callback = None
        self.switch_result = SwitchResult(available=False)
        self.switch_map = Mock(side_effect=lambda path, timeout=5.0: self.switch_result)

    def set_doc_changed_callback(self, callback) -> None:
        """Store the callback notified when the harness document is replaced."""
        self._callback = callback

    def get_doc(self):
        """Return the document currently held by the harness node."""
        return self._state['_topo_doc']

    def set_doc(self, doc) -> None:
        """Replace the harness document and notify the registered callback."""
        self._state['_topo_doc'] = doc
        if self._callback is not None:
            self._callback(doc)

    def publish(self) -> None:
        """Send the document dictionary to the harness publisher for assertions."""
        self._state['_topo_map_pub'].publish(self.get_doc().to_dict())


def attach_topology(node, maps_dir):
    """Wire the fake domain, a real app service on maps_dir and a real view model onto node."""
    domain = FakeTopologyDomain(node)
    store = TopoMapStore(str(maps_dir))
    app = TopologyApplicationService(domain, store)
    # Populate the namespace used by the methods compiled into the test harness.
    vars(node).update(
        _topo_map_pub=Mock(),
        _topo_domain=domain,
        topo_store=store,
        _topo_app_service=app,
        _topo_vm=TopologyViewModel(app),
    )
    return node


def patch_store_io(testcase):
    """Wrap the map store's file boundaries in mocks that still do the real I/O.

    Returns a dict keyed 'parse_topo_yaml', 'dump_topo_yaml' and 'os' (with .makedirs) so
    tests can assert on calls or set side_effect to inject failures.
    """
    parse = Mock(side_effect=topo_map_store.parse_topo_yaml)
    dump = Mock(side_effect=topo_map_store.dump_topo_yaml)
    makedirs = Mock(side_effect=os.makedirs)
    fake_os = SimpleNamespace(path=os.path, makedirs=makedirs)
    testcase.enterContext(patch.object(topo_map_store, 'parse_topo_yaml', parse))
    testcase.enterContext(patch.object(topo_map_store, 'dump_topo_yaml', dump))
    testcase.enterContext(patch.object(topo_map_store, 'os', fake_os))
    return {'parse_topo_yaml': parse, 'dump_topo_yaml': dump, 'os': fake_os}
