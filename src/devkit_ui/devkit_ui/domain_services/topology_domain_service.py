from __future__ import annotations

import json
import threading

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from devkit_ui.models import TopoDoc, TopoNode
from devkit_ui.parse import parse_topo_json
from devkit_ui.ros_gateway import RosGateway
from devkit_ui.topo_results import SwitchResult

try:
    from topological_navigation_msgs.srv import WriteTopologicalMap
except ImportError:  # topological navigation not installed (e.g. unit tests)
    WriteTopologicalMap = None

# QoS for topology map: reliable, transient-local so new subscribers get last map
TMAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST
)


class TopologyDomainService:
    """Owns the in-memory topology document and manages its ROS lifecycle.

    Responsibilities:
    - Store and mutate the TopoDoc
    - Subscribe to /topological_map_2 (incoming map updates)
    - Publish to /topological_map_2 (outgoing updates)
    - Provide query and mutation interfaces
    """

    def __init__(self, ros_gateway: RosGateway) -> None:
        """Initialize the topology domain service.

        Parameters:
            ros_gateway: ROS gateway for subscriptions/publications.
        """
        self._doc: TopoDoc | None = None
        self._ros = ros_gateway
        self._on_doc_changed = None  # Set via set_doc_changed_callback()
        self._topo_map_pub = None

        # Subscribe to map updates and create publisher
        self._ros.create_subscription(String, '/topological_map_2', self._on_topo_map, TMAP_QOS)
        self._topo_map_pub = self._ros.create_publisher(String, '/topological_map_2', TMAP_QOS)
        self._switch_cli = (
            self._ros.create_client(
                WriteTopologicalMap, '/topological_map_manager2/switch_topological_map')
            if WriteTopologicalMap is not None else None)

    def set_doc_changed_callback(self, callback) -> None:
        """Register a callback to be invoked whenever the doc changes.

        Called by the application service to wire the callback.

        Parameters:
            callback: Callable(doc: TopoDoc | None) invoked on doc changes.
        """
        self._on_doc_changed = callback

    def _on_topo_map(self, msg: String) -> None:
        """Handle incoming topology map from ROS."""
        try:
            self._doc = parse_topo_json(msg.data)
            if self._on_doc_changed:
                self._on_doc_changed(self._doc)
        except Exception as e:
            # Log error but don't crash; keep existing doc
            self._ros.get_logger().error(f"Failed to parse /topological_map_2: {e}")

    def publish(self) -> None:
        """Publish the current document on /topological_map_2.

        Synchronous, so rapid edits are published in call order. Raises if the
        publisher fails; callers decide whether that matters.
        """
        if self._topo_map_pub is None or self._doc is None:
            return
        msg = String()
        msg.data = json.dumps(self._doc.to_dict(), ensure_ascii=False)
        self._topo_map_pub.publish(msg)

    def switch_map(self, path: str, timeout: float = 5.0) -> SwitchResult:
        """Ask the map manager to load the map file at path. Blocks up to timeout seconds.

        Call from a worker thread, never from the ROS executor or UI loop.
        """
        if self._switch_cli is None:
            return SwitchResult(available=False)
        req = WriteTopologicalMap.Request()
        req.filename = path
        req.no_alias = True
        done = threading.Event()
        result = [None]

        def _on_done(future) -> None:
            result[0] = future.result()
            done.set()

        self._switch_cli.call_async(req).add_done_callback(_on_done)
        done.wait(timeout=timeout)
        resp = result[0]
        if resp is None:
            return SwitchResult(available=True, success=False, message='timeout')
        return SwitchResult(available=True, success=bool(resp.success),
                            message=getattr(resp, 'message', ''))

    def get_doc(self) -> TopoDoc | None:
        """Return the current topology document."""
        return self._doc

    def has_node(self, name: str) -> bool:
        """Check if a node exists in the current document."""
        if self._doc is None:
            return False
        return self._doc.has_node(name)

    def get_node(self, name: str) -> TopoNode | None:
        """Get a node by name from the current document."""
        if self._doc is None:
            return None
        return self._doc.get_node(name)

    def add_node(self, node: TopoNode) -> None:
        """Add a node to the topology document.

        Parameters:
            node: TopoNode to add.

        Raises:
            ValueError: If no document is loaded or node already exists.
        """
        if self._doc is None:
            raise ValueError('No topology document loaded')
        if self._doc.has_node(node.name):
            raise ValueError(f"Node '{node.name}' already exists")

        self._doc.add_node(node)
        self.publish()

    def remove_node(self, name: str) -> None:
        """Remove a node from the topology document.

        Parameters:
            name: Name of the node to remove.

        Raises:
            ValueError: If no document is loaded or node doesn't exist.
        """
        if self._doc is None:
            raise ValueError('No topology document loaded')
        if not self._doc.has_node(name):
            raise ValueError(f"Node '{name}' not found")

        self._doc.remove_node(name)
        self.publish()

    def set_doc(self, doc: TopoDoc | None) -> None:
        """Set the topology document (used for initialization or reloading).

        Parameters:
            doc: New TopoDoc to use, or None to clear.
        """
        self._doc = doc
        if self._on_doc_changed:
            self._on_doc_changed(self._doc)
