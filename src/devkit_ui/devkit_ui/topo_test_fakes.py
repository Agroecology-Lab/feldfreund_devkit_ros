"""Test doubles that stand in for the topology services on a harness node.

The harnesses exec single NiceGuiNode methods in isolation, so they have no ROS
publisher. ``attach_topology`` gives them a real TopologyViewModel on top of a
fake application service whose publish() lands on ``node._topo_map_pub``, which
is the mock the tests assert on.
"""
from unittest.mock import Mock

from devkit_ui.view_models.topology_view_model import TopologyViewModel


class FakeTopologyAppService:
    """Application-service stand-in that keeps the document on the harness node."""

    def __init__(self, node) -> None:
        self._node = node
        self._callback = None

    def register_doc_changed_callback(self, callback) -> None:
        self._callback = callback

    def set_doc(self, doc) -> None:
        self._node._topo_doc = doc
        if self._callback is not None:
            self._callback(doc)

    def publish(self) -> None:
        self._node._topo_map_pub.publish(self._node._topo_doc.to_dict())


def attach_topology(node):
    """Wire a fake topology app service, a real view model and a publisher mock onto node."""
    node._topo_map_pub = Mock()
    node._topo_app_service = FakeTopologyAppService(node)
    node._topo_vm = TopologyViewModel(node._topo_app_service)
    return node
