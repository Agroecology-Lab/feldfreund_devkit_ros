from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from devkit_ui.application_services.topology_application_service import (
        TopologyApplicationService,
    )
    from devkit_ui.models import TopoDoc, TopoNode


class TopologyViewModel:
    def __init__(self, app_service: TopologyApplicationService) -> None:
        """Initialize the topology view model.

        Parameters:
            app_service: TopologyApplicationService instance for operations.
        """
        self._app_service = app_service
        self.topo_doc: TopoDoc | None = None
        self.selected_node: str | None = None
        self.current_node: str = '—'

        # Register this view model's callback with the domain via app service
        # This ensures doc changes sync automatically to the UI state
        self._app_service.register_doc_changed_callback(self._on_doc_changed)

    def set_selected_node(self, name: str | None) -> None:
        """Set the selected node name, or clear the selection."""
        self.selected_node = name

    def set_current_node(self, name: str) -> None:
        """Update the node the robot is currently localised on."""
        self.current_node = name

    def _on_doc_changed(self, topo_doc: TopoDoc | None) -> None:
        """Callback invoked when topology document changes.

        Keeps the view model's doc mirror and UI state in sync with the domain.
        Called when:
        - Domain receives ROS updates from other services
        - Local mutations are published and echo back via subscription
        - Initial document is set during app initialization

        Parameters:
            topo_doc: Updated TopoDoc, or None if cleared.
        """
        self.update_doc(topo_doc)

    def update_doc(self, topo_doc: TopoDoc | None) -> None:
        """Replace the topology document and clear stale selection state.

        Called by _on_doc_changed callback when doc changes externally
        or locally via mutations.
        """
        self.topo_doc = topo_doc
        if self.selected_node is not None and (
            topo_doc is None or not topo_doc.has_node(self.selected_node)
        ):
            self.selected_node = None

    def add_node(self, node: TopoNode) -> None:
        """Add a node to the topology via the application service.

        Parameters:
            node: TopoNode to add.
        """
        self._app_service.add_node(node)

    def remove_node(self, name: str) -> None:
        """Remove a node from the topology via the application service.

        Parameters:
            name: Name of the node to remove.
        """
        self._app_service.remove_node(name)

    def has_node(self, name: str) -> bool:
        """Check if a node exists in the current topology.

        Parameters:
            name: Name of the node to check.

        Returns:
            True if node exists, False otherwise.
        """
        return self._app_service.has_node(name)

    def get_node(self, name: str) -> TopoNode | None:
        """Get a node by name from the current topology.

        Parameters:
            name: Name of the node to retrieve.

        Returns:
            TopoNode if found, None otherwise.
        """
        return self._app_service.get_node(name)
