import copy
from collections.abc import Iterator
from datetime import datetime
from typing import NamedTuple

# TODO(robbrit): These position types are probably defined somewhere else and we can use those.

EdgeID = str
NodeID = str

class Vector4(NamedTuple):
    """Represents a 4D vector."""
    x: float
    y: float
    z: float = 0.0
    w: float = 1.0

class Vector3(NamedTuple):
    """Represents a 3D vector."""
    x: float
    y: float
    z: float = 0.0

class Vector2(NamedTuple):
    """Represents a 2D vector."""
    x: float
    y: float

class TopoPose:
    """Represents a point's pose in 3D space."""
    def __init__(self, **kwargs):
        if 'orientation' not in kwargs:
            kwargs['orientation'] = Vector4(x=0.0, y=0.0, z=0.0, w=1.0)
        if 'position' not in kwargs:
            kwargs['position'] = Vector3(x=kwargs.get('x', 0.0), y=kwargs.get('y', 0.0), z=0.0)

        self._orientation: Vector4 = kwargs['orientation']
        self._position: Vector3 = kwargs['position']

    def to_dict(self) -> dict:
        return {
            'orientation': self._orientation._asdict(),
            'position': self._position._asdict(),
        }

    @property
    def orientation(self) -> Vector4:
        return self._orientation

    @property
    def position(self) -> Vector3:
        return self._position

class TopoProperties(NamedTuple):
    """Contains a set of properties for a topo node."""
    xy_goal_tolerance: float = 0.0
    yaw_goal_tolerance: float = 0.0
    dropped_by: str = ''
    timestamp: datetime | None = None
    row_id: str = ''
    row_role: str = ''
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    gps_fix_type: int = 0
    gps_hdop: float | None = None

class TopoEdge(NamedTuple):
    """Represents an edge between two topo nodes.

    Note that these are not standalone, they are always part of a TopoNode in order to know what
    the two nodes are that are being connected.
    """
    action: str
    edge_id: EdgeID
    node: NodeID

class TopoNode:
    """Represents a topo node."""

    def __init__(
        self,
        name: NodeID,
        pointset: str | None = None,
        nav_frame: str | None = None,
        edges: list[TopoEdge | str] | None = None,
        x: float | None = None,
        y: float | None = None,
        pose: TopoPose | None = None,
        properties: TopoProperties | None = None,
        verts: list[Vector2] | None = None,
        meta: dict | None = None,
    ) -> None:
        """Constructs a new topo node.

        For convenience you can pass in an x/y pair instead of a full pose.
        """


        if pose is not None:
            if x is not None or y is not None:
                raise ValueError("cannot pass x and y if pose is provided")
            self._pose = pose
        else:
            self._pose = TopoPose(position=Vector3(x=x, y=y))

        self._name: NodeID = name
        self._pointset: str = pointset or name
        self._nav_frame: str = nav_frame or name
        self._edges = {}
        for edge in edges or []:
            if isinstance(edge, TopoEdge):
                self._edges[edge.node] = edge
            else:
                self._edges[edge] = TopoEdge(action='', edge_id=f'{self._name}_{edge}', node=edge)

        self._properties: TopoProperties = properties or TopoProperties()
        self._verts: list[Vector2] = verts or []
        self._meta: dict = meta or {}

    @property
    def name(self) -> NodeID:
        return self._name

    @property
    def edges(self) -> Iterator[TopoEdge]:
        return self._edges.values()

    @property
    def meta(self) -> dict:
        return self._meta

    @property
    def x(self) -> float:
        return self._pose.position.x

    @property
    def y(self) -> float:
        return self._pose.position.y

    def ensure_meta(self, map_name: str) -> None:
        """Ensures that the metadata on the node is valid."""
        self._meta.setdefault('map', map_name)
        self._meta.setdefault('node', self._name)
        self._meta.setdefault('pointset', map_name)

    def add_metadata(self, **kwargs: dict) -> None:
        self._meta.update(**kwargs)

    def add_edge(self, edge: TopoEdge | NodeID, action: str | None = None) -> None:
        if isinstance(edge, NodeID):
            edge = TopoEdge(action=action or '', edge_id=f'{self._name}_{edge}', node=edge)

        if edge.node == self._name:
            return

        self._edges[edge.node] = edge

    def remove_edge(self, node_id: NodeID) -> None:
        self.remove_edges({node_id})

    def is_connected_to(self, node_id: NodeID) -> bool:
        return node_id in self._edges

    def remove_edges(self, node_ids: set[NodeID]) -> None:
        for node_id in node_ids:
            if node_id not in self._edges:
                continue
            self._edges.pop(node_id)

    def to_dict(self) -> dict:
        return {
            'node': {
                'name': self._name,
                'pointset': self._pointset,
                'nav_frame': self._nav_frame,
                'edges': [edge._asdict() for edge in self._edges.values()],
                'pose': self._pose.to_dict(),
                'properties': self._properties._asdict(),
                'verts': [vert._asdict() for vert in self._verts],
            },
            'meta': self._meta,
        }

    def patch_role(self, role: str) -> None:
        self._properties['row_role'] = role
        self._meta['row_role'] = role

class TopoDoc:
    """Represents a topo document."""

    def __init__(
        self,
        name: str,
        nodes: list[TopoNode] | list[dict] | None = None,
        metric_map: str | None = None,
        pointset: str | None = None,
        meta: dict | None = None,
        actions: dict | None = None,
        definitions: dict | None = None,
        transformation: dict | None = None,
    ) -> None:
        self._nodes: dict[NodeID, TopoNode] = {}
        for node in nodes or []:
            if isinstance(node, TopoNode):
                self._nodes[node.name] = node
            else:
                topo_node = TopoNode(**node)
                self._nodes[topo_node.name] = topo_node

        # TODO(rob): Since the nodes here are specified by the caller, it's possible there are edge
        # inconsistenceies. We should validate the edges here.

        self._name: str = name
        self._metric_map: str = metric_map or name
        self._pointset: str = pointset or name

        # TODO(robbrit): Define these ones properly.
        self._meta: dict = meta or {}
        self._actions: dict = actions or {}
        self._definitions: dict = definitions or {}

        # topological_navigation's localisation2.py does a bare
        # tmap["transformation"]["topo_frame_id"] lookup with no fallback,
        # so this key must always be present on any doc we hand off,
        # whether freshly constructed, parsed from disk, or cloned.
        self._transformation: dict = dict(transformation or {})
        self._transformation.setdefault('topo_frame_id', 'map')

    def ensure_meta(self, map_name: str | None) -> None:
        """Ensures that the metadata on the document is valid."""
        map_name = map_name or self._name

        for node in self._nodes.values():
            node.ensure_meta(map_name)

    def has_node(self, name: NodeID) -> bool:
        return name in self._nodes

    def get_node(self, name: NodeID) -> TopoNode:
        return self._nodes[name]

    @property
    def nodes(self) -> Iterator[TopoNode]:
        return self._nodes.values()

    @property
    def name(self) -> str:
        return self._name

    @property
    def actions(self) -> dict:
        return self._actions

    @property
    def definitions(self) -> dict:
        return self._definitions

    def seed_actions(self, actions: dict, definitions: dict) -> None:
        """Backfill actions/definitions from a template doc (e.g. the
        installed mixed_actions_map.yaml) when this doc has none of its
        own. Used when clearing/re-seeding a map that was itself loaded
        from an actions-less authored source, so navigation2.py always
        gets a real NavigateToPose/limbic_row_follow action config
        instead of silently running with none."""
        self._actions = copy.deepcopy(actions)
        self._definitions = copy.deepcopy(definitions)

    @property
    def transformation(self) -> dict:
        return self._transformation

    def add_node(self, node: TopoNode) -> None:
        self._nodes[node.name] = node

        for edge in node.edges:
            node_id = edge.node

            self._nodes[node_id].add_edge(TopoEdge(
                action=edge.action,
                edge_id=edge.edge_id,
                node=node.name,
            ))

    def insert_node(self, node: TopoNode) -> None:
        """Add a node exactly as given, with no reverse-edge backfill.

        Unlike add_node(), which assumes it's dropping a single node onto
        an existing graph and back-fills the reverse edge on each
        neighbour, this is for callers that pass in nodes whose edges are
        already fully and correctly wired (in both directions, where
        that's wanted) before the call — e.g. a batch of new nodes that
        reference each other. add_node() would KeyError on a sibling not
        yet inserted, and would silently add reverse edges the caller
        didn't ask for onto pre-existing nodes.
        """
        self._nodes[node.name] = node

    def remove_node(self, name: NodeID) -> None:
        self.remove_nodes({name})

    def remove_nodes(self, names: set[NodeID]) -> None:
        for name in names:
            self._nodes.pop(name)

        for node in self.nodes:
            node.remove_edges(names)

    def to_dict(self) -> dict:
        return {
            'name': self._name,
            'nodes': [node.to_dict() for node in self._nodes.values()],
            'metric_map': self._metric_map,
            'pointset': self._pointset,
            'meta': self._meta,
            'actions': self._actions,
            'definitions': self._definitions,
            'transformation': self._transformation,
        }

    def clone_empty(self, map_name: str | None = None) -> 'TopoDoc':
        """Clones the document and returns a new document with no nodes.
        Retains metadata, actions, definitions, and transformation."""
        return TopoDoc(
            name=map_name or self._name,
            metric_map=self._metric_map,
            pointset=self._pointset,
            meta=copy.deepcopy(self._meta),
            actions=copy.deepcopy(self._actions),
            definitions=copy.deepcopy(self._definitions),
            transformation=copy.deepcopy(self._transformation),
            nodes=[],
        )
