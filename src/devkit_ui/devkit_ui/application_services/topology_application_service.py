from __future__ import annotations

import copy
import logging
import re
import threading
from collections.abc import Callable
from typing import TYPE_CHECKING

from devkit_ui.domain_services.topo_map_store import TopoMapStore
from devkit_ui.models import TopoDoc, TopoNode, TopoPose
from devkit_ui.topo_defaults import default_actions, default_definitions
from devkit_ui.topo_results import PersistResult

if TYPE_CHECKING:
    from devkit_ui.domain_services.topology_domain_service import TopologyDomainService


_LOG = logging.getLogger(__name__)

MAP_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')


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

    def __init__(self, domain_service: TopologyDomainService,
                 store: TopoMapStore | None = None) -> None:
        """Initialize the topology application service.

        Parameters:
            domain_service: TopologyDomainService to delegate operations to.
            store: Map file storage; defaults to /workspace/maps.
        """
        self._domain = domain_service
        self._store = store or TopoMapStore()
        self._lock = threading.Lock()

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

    # ── persistence ──────────────────────────────────────────────────────────
    # These block on file and service I/O: call them from a worker thread.

    def persist_and_reload(self, modify: Callable[[TopoDoc], None]) -> PersistResult:
        """Apply modify to the saved map, save it, and make it live.

        Edits the file on disk when there is one, otherwise a copy of the live map seeded
        with default actions if it has none. Raises on read, modify or write errors, and
        leaves the live document untouched in that case.
        """
        with self._lock:
            live = self._require_doc()
            name = live.name
            if self._store.exists(name):
                file_doc = self._store.load(name)
            else:
                file_doc = copy.deepcopy(live)
                if not file_doc.actions:
                    file_doc.seed_actions(default_actions(), default_definitions())

            modify(file_doc)

            # Backfill per-node meta the tmap schema requires (hand-written nodes, F2C rows).
            # Must run after modify(), or newly added nodes are never backfilled.
            file_doc.ensure_meta(name)

            self._store.save(file_doc, name)

        return self._go_live(name, file_doc)

    def drop_node(self, node: TopoNode,
                  on_saved: Callable[[], None] | None = None) -> PersistResult:
        """Save a newly dropped node into the map file and make it live.

        Creates a missing map from repo defaults. Only edges to nodes already on disk are
        saved, and no reverse edges are added. If the name is already on disk nothing is
        written and the result is 'skipped'. on_saved runs after the file is written and
        the live document replaced, before the map manager is asked to switch.
        """
        with self._lock:
            live = self._require_doc()
            name = live.name
            if self._store.exists(name):
                file_doc = self._store.load(name)
            else:
                file_doc = live.clone_empty(name)
                file_doc.seed_actions(default_actions(), default_definitions())

            existing = {n.name for n in file_doc.nodes}
            if node.name in existing:
                return PersistResult('skipped', f'{node.name} already in file')

            saved = copy.deepcopy(node)
            saved.remove_edges({edge.node for edge in saved.edges if edge.node not in existing})
            file_doc.insert_node(saved)
            self._store.save(file_doc, name)

        return self._go_live(name, file_doc, on_saved)

    def patch_node_role(self, node_name: str, role: str) -> None:
        """Set a node's role in the saved map. Does nothing if there is no file."""
        with self._lock:
            live = self._domain.get_doc()
            if live is None or not self._store.exists(live.name):
                return
            doc = self._store.load(live.name)
            if doc.has_node(node_name):
                doc.get_node(node_name).patch_role(role)
            self._store.save(doc, live.name)

    def save_map_as(self, name: str) -> str:
        """Save a named copy of the map without touching the live one.

        Prefers the saved file over memory, refuses to overwrite, and returns
        'saved → <name>' or an 'ERROR: ...' status.
        """
        with self._lock:
            name = (name or '').strip()
            live = self._domain.get_doc()
            if not live:
                return 'ERROR: map not loaded'
            if not MAP_NAME_RE.match(name):
                return 'ERROR: use letters, digits, _ or - (max 64, start with a letter or digit)'
            if name == live.name:
                return 'ERROR: that is the live map name'
            if self._store.exists(name):
                return f'ERROR: {name} already exists'
            try:
                src = self._store.load(live.name) if self._store.exists(live.name) else live
                if not any(True for _ in src.nodes):
                    return 'ERROR: map has no nodes'
                self._store.save(src.renamed(name), name)
                _LOG.info('save_map_as: saved %s', self._store.path(name))
                return f'saved → {name}'
            except Exception as e:  # pylint: disable=broad-except
                _LOG.error('save_map_as failed: %s', e)
                return f'ERROR: {e}'

    def archive_and_clear(self) -> str:
        """Archive the saved map as <name>_<N>, then replace it with an empty default map.

        Returns the archive name. Raises on read or write errors; writes already made are
        not rolled back and the live document is only replaced after both writes succeed.
        A failure to publish is ignored.
        """
        with self._lock:
            live = self._require_doc()
            name = live.name
            archive = self._store.next_archive_name(name)

            # Archive what is on disk, not just memory.
            on_disk = self._store.load(name) if self._store.exists(name) else copy.deepcopy(live)
            self._store.save(on_disk, archive)

            # A cleared map is a new map: always start from repo defaults.
            empty = live.clone_empty(name)
            empty.seed_actions(default_actions(), default_definitions())
            self._store.save(empty, name)

        self._domain.set_doc(empty)
        try:
            self._domain.publish()
        except Exception:  # pylint: disable=broad-except
            pass
        return archive

    def _require_doc(self) -> TopoDoc:
        """Return the live document or raise ValueError when no map is loaded."""
        doc = self._domain.get_doc()
        if doc is None:
            raise ValueError('map not loaded')
        return doc

    def _go_live(self, name: str, file_doc: TopoDoc,
                 on_saved: Callable[[], None] | None = None) -> PersistResult:
        """Replace the live document with the saved one and get the nav stack to load it."""
        self._domain.set_doc(file_doc)
        if on_saved is not None:
            on_saved()
        switch = self._domain.switch_map(self._store.path(name))
        if switch.available and switch.success:
            return PersistResult('live')
        # No service, or it failed: publish so the nav stack still sees the new map.
        self._domain.publish()
        if not switch.available:
            return PersistResult('no_srv')
        return PersistResult('switch_failed', switch.message)
