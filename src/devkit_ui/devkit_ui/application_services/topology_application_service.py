from __future__ import annotations

from typing import TYPE_CHECKING

from devkit_ui.models import TopoDoc, TopoNode, TopoPose

if TYPE_CHECKING:
    from devkit_ui.domain_services.topology_domain_service import TopologyDomainService


def create_default_doc() -> TopoDoc:
    """Return the demo map shown before any real map arrives (pure data, no ROS)."""
    return TopoDoc(
        name='mixed_test_map',
        nodes=[
            TopoNode(name='N1', pose=TopoPose(x=0.0, y=0.0), edges=['N2'], meta={}),
            TopoNode(name='N2', pose=TopoPose(x=3.0, y=0.0), edges=['N1', 'N3'], meta={}),
            TopoNode(name='N3', pose=TopoPose(x=6.0, y=0.0), edges=['N2', 'N4'], meta={}),
            TopoNode(name='N4', pose=TopoPose(x=9.0, y=0.0), edges=['N3', 'N5'], meta={}),
            TopoNode(name='N5', pose=TopoPose(x=12.0, y=0.0), edges=['N4', 'N6'], meta={}),
            TopoNode(name='N6', pose=TopoPose(x=15.0, y=0.0), edges=['N5'], meta={}),
        ],
    )


class TopologyApplicationService:
    """Orchestrates topology domain operations.

    Responsibilities:
    - Provide application-level interface to topology operations
    - Delegate to TopologyDomainService for mutations and queries
    - Handle persistence to file and service coordination
    """

    def __init__(self, domain_service: TopologyDomainService) -> None:
        """Initialize the topology application service.

        Parameters:
            domain_service: TopologyDomainService to delegate operations to.
        """
        self._domain = domain_service

    def register_doc_changed_callback(self, callback) -> None:
        """Register a callback with the domain service for doc changes.

        Parameters:
            callback: Callable(doc: TopoDoc | None) to invoke on doc changes.
        """
        self._domain.set_doc_changed_callback(callback)

    def initialize_with_default(self) -> None:
        """Initialize the topology with a default demo document.

        Useful for first-run UI state when no map file is available.
        Triggers the on_doc_changed callback to sync the UI.
        """
        self._domain.set_doc(create_default_doc())

    def set_doc(self, doc: TopoDoc | None) -> None:
        """Replace the live document (e.g. after a persist) without publishing."""
        self._domain.set_doc(doc)

    def publish(self) -> None:
        """Publish the live document so the topo nav stack picks it up."""
        self._domain.publish()

    def get_doc(self) -> TopoDoc | None:
        """Get the current topology document from the domain.

        Returns:
            Current TopoDoc or None if not loaded.
        """
        return self._domain.get_doc()

    def has_node(self, name: str) -> bool:
        """Check if a node exists in the current topology.

        Parameters:
            name: Name of the node to check.

        Returns:
            True if node exists, False otherwise.
        """
        return self._domain.has_node(name)

    def get_node(self, name: str) -> TopoNode | None:
        """Get a node by name from the current topology.

        Parameters:
            name: Name of the node to retrieve.

        Returns:
            TopoNode if found, None otherwise.
        """
        return self._domain.get_node(name)

    def add_node(self, node: TopoNode) -> None:
        """Add a node to the topology and publish it.

        Parameters:
            node: TopoNode to add.

        Raises:
            ValueError: If no document is loaded or node already exists.
        """
        self._domain.add_node(node)

    def remove_node(self, name: str) -> None:
        """Remove a node from the topology and publish the change.

        Parameters:
            name: Name of the node to remove.

        Raises:
            ValueError: If no document is loaded or node doesn't exist.
        """
        self._domain.remove_node(name)
