# pylint: disable=duplicate-code
"""
ui_node.py — Sowbot web cockpit on :80
"""

import json
import os
import re
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from nicegui import app, ui, ui_run
from nicegui.events import ClickEventArguments
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, Duration, HistoryPolicy,
    LivelinessPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, String

_TOPO_SRV_OK = False
try:
    from topological_navigation_msgs.srv import WriteTopologicalMap
    _TOPO_SRV_OK = True
except ImportError:
    pass

_ACTION_OK = False
try:
    from rclpy.action import ActionClient
    from topological_navigation_msgs.action import GotoNode
    _ACTION_OK = True
except ImportError:
    pass

SAFETY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)

TMAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

_ROW_ACTION = 'limbic_row_follow'
_NAV_ACTION = 'navigate_to_pose'
_NAME_RE = re.compile(r'^[A-Z0-9_]+$')

# ── Global CSS ────────────────────────────────────────────────────────────────

_GLOBAL_CSS = """
<style>
:root {
  --bg:        #f6f8fa;
  --bg-card:   #ffffff;
  --border:    #d0d7de;
  --txt:       #24292f;
  --txt-dim:   #57606a;
  --txt-muted: #8c959f;
  --green:     #1a7f37;
  --amber:     #9a6700;
  --red:       #cf222e;
  --blue:      #0969da;
}
body, .nicegui-content { background: var(--bg) !important; color: var(--txt); }
.q-header { background: var(--bg-card) !important;
            border-bottom: 1px solid var(--border) !important;
            box-shadow: none !important; color: var(--txt) !important; }
.q-card   { background: var(--bg-card) !important;
            border: 1px solid var(--border) !important;
            box-shadow: none !important; }
.q-separator { background: var(--border) !important; }
.q-tab__label { color: var(--txt) !important; }
.q-tab--active .q-tab__label { color: var(--blue) !important; font-weight: 600; }
.q-tab-panels { background: var(--bg) !important; }

.sec-label {
  font-size: 10px; font-weight: 600; letter-spacing: 0.08em;
  text-transform: uppercase; color: var(--txt-muted);
  font-family: 'Courier New', monospace; margin-bottom: 2px;
}
.dot-ok   { display:inline-block;width:7px;height:7px;border-radius:50%;background:#1a7f37;margin-right:6px; }
.dot-warn { display:inline-block;width:7px;height:7px;border-radius:50%;background:#9a6700;margin-right:6px; }
.dot-off  { display:inline-block;width:7px;height:7px;border-radius:50%;background:#d0d7de;margin-right:6px; }

.node-item {
  display:block; padding:4px 8px; border-radius:4px;
  font-family:'Courier New',monospace; font-size:11px;
  cursor:pointer; border-left:3px solid transparent;
  color:var(--txt-dim); transition:background 0.1s;
}
.node-item:hover { background:#f6f8fa; color:var(--txt); }
.node-item.sel   { background:#fff8c5; color:var(--amber); border-left-color:var(--amber); }
.node-item.row   { color:var(--blue); border-left-color:#d8e8fd; }
.node-item.sel.row { background:#fff8c5; color:var(--amber); border-left-color:var(--amber); }

.topo-node:hover { filter:brightness(0.85); cursor:pointer; }

.nav-sidebar {
  width:220px; flex-shrink:0;
  display:flex; flex-direction:column; gap:6px; align-self:stretch;
}

.estop-btn {
  min-width:120px !important; min-height:52px !important;
  font-size:15px !important; font-weight:700 !important;
}
</style>
"""

# ── map parser ────────────────────────────────────────────────────────────────

def _parse_topo_json(json_str: str) -> tuple[dict[str, dict], dict]:
    doc = json.loads(json_str)
    nodes: dict[str, dict] = {}
    for entry in doc.get('nodes', []):
        n     = entry['node']
        name  = n['name']
        pos   = n['pose']['position']
        edges = [e['node'] for e in n.get('edges', []) if 'node' in e]
        meta  = dict(entry.get('meta', {}))
        props = n.get('properties', {})
        for key in ('dropped_by', 'timestamp', 'row_id', 'row_role',
                    'gps_lat', 'gps_lon', 'gps_fix_type', 'gps_hdop'):
            if key in props and key not in meta:
                meta[key] = props[key]
        nodes[name] = {'x': float(pos['x']), 'y': float(pos['y']),
                       'edges': edges, 'meta': meta}
    return nodes, doc


def _demo_nodes() -> dict[str, dict]:
    return {
        'N1': {'x':  0.0, 'y': 0.0, 'edges': ['N2'],       'meta': {}},
        'N2': {'x':  3.0, 'y': 0.0, 'edges': ['N1', 'N3'], 'meta': {}},
        'N3': {'x':  6.0, 'y': 0.0, 'edges': ['N2', 'N4'], 'meta': {}},
        'N4': {'x':  9.0, 'y': 0.0, 'edges': ['N3', 'N5'], 'meta': {}},
        'N5': {'x': 12.0, 'y': 0.0, 'edges': ['N4', 'N6'], 'meta': {}},
        'N6': {'x': 15.0, 'y': 0.0, 'edges': ['N5'],       'meta': {}},
    }


# ── SVG renderer ──────────────────────────────────────────────────────────────

_SVG_W, _SVG_H = 900, 480
_MARGIN, _NODE_R = 56, 17


def _build_svg(nodes: dict, selected: Optional[str], current: Optional[str]) -> str:
    if not nodes:
        return (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
                f'style="background:#f6f8fa;border-radius:4px;border:1px solid #d0d7de">'
                f'<text x="{_SVG_W//2}" y="{_SVG_H//2}" text-anchor="middle" '
                f'fill="#8c959f" font-family="Courier New" font-size="13">No map loaded</text>'
                f'</svg>')

    xs = [n['x'] for n in nodes.values()]
    ys = [n['y'] for n in nodes.values()]
    dx = (max(xs) - min(xs)) or 1.0
    dy = (max(ys) - min(ys)) or 1.0
    x_min, y_min = min(xs), min(ys)

    def tx(x): return _MARGIN + (x - x_min) / dx * (_SVG_W - 2 * _MARGIN)
    def ty(y): return _SVG_H - _MARGIN - (y - y_min) / dy * (_SVG_H - 2 * _MARGIN)

    parts: list[str] = [f'<rect width="{_SVG_W}" height="{_SVG_H}" fill="#f6f8fa" rx="4"/>']

    drawn: set = set()
    for name, nd in nodes.items():
        for tgt in nd['edges']:
            key = tuple(sorted([name, tgt]))
            if key in drawn or tgt not in nodes:
                continue
            drawn.add(key)
            is_row_edge = (nd.get('meta', {}).get('row_id') is not None
                           or nodes[tgt].get('meta', {}).get('row_id') is not None)
            parts.append(
                f'<line x1="{tx(nd["x"]):.1f}" y1="{ty(nd["y"]):.1f}" '
                f'x2="{tx(nodes[tgt]["x"]):.1f}" y2="{ty(nodes[tgt]["y"]):.1f}" '
                f'stroke="{"#d8e8fd" if is_row_edge else "#d0d7de"}" '
                f'stroke-width="{"2" if is_row_edge else "1"}" stroke-linecap="round"/>')

    for name, nd in nodes.items():
        cx, cy = tx(nd['x']), ty(nd['y'])
        is_cur = name == current
        is_sel = name == selected
        is_row = nd.get('meta', {}).get('row_id') is not None

        if is_cur:   fill, stroke, sw = '#dafbe1', '#1a7f37', 2
        elif is_sel: fill, stroke, sw = '#fff8c5', '#9a6700', 2
        elif is_row: fill, stroke, sw = '#d8e8fd', '#0969da', 1
        elif nd.get('meta', {}).get('dropped_by'): fill, stroke, sw = '#f6f8fa', '#8c959f', 1
        else:        fill, stroke, sw = '#ffffff', '#d0d7de', 1

        label_col = ('#1a7f37' if is_cur else '#9a6700' if is_sel
                     else '#0969da' if is_row else '#57606a')

        if is_cur:
            parts.append(
                f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R+6}" '
                f'fill="none" stroke="#1a7f37" stroke-width="1" opacity="0.4">'
                f'<animate attributeName="r" values="{_NODE_R+6};{_NODE_R+13}" '
                f'dur="2s" repeatCount="indefinite"/>'
                f'<animate attributeName="opacity" values="0.4;0" '
                f'dur="2s" repeatCount="indefinite"/></circle>')

        parts.append(
            f'<circle class="topo-node" data-node="{name}" '
            f'cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

        if is_row:
            parts.append(
                f'<text x="{cx:.1f}" y="{cy+4:.1f}" text-anchor="middle" fill="#0969da" '
                f'font-family="Courier New" font-size="8" font-weight="700" '
                f'style="pointer-events:none">'
                f'{nd["meta"].get("row_role","?")[0].upper()}{nd["meta"].get("row_id","")}'
                f'</text>')

        parts.append(
            f'<text x="{cx:.1f}" y="{cy+_NODE_R+12:.1f}" text-anchor="middle" '
            f'fill="{label_col}" font-family="Courier New" font-size="9" '
            f'style="pointer-events:none">{name}</text>')

    return (f'<svg id="topo-map" width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg">{"".join(parts)}</svg>')


# ── Fields2Cover geometry helpers (module-level so cpu_bound can pickle them) ─

def _f2c_latlon_to_xy(lat: float, lon: float,
                      lat0: float, lon0: float) -> tuple[float, float]:
    import math
    R = 6_378_137.0
    x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R
    return x, y


def _f2c_xy_to_latlon(x: float, y: float,
                      lat0: float, lon0: float) -> tuple[float, float]:
    import math
    R   = 6_378_137.0
    lat = lat0 + math.degrees(y / R)
    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
    return lat, lon


def _run_f2c(corners_ll: list, tool_width: float,
             angle_deg: float) -> list:
    """
    Pure function — no self, safe to pickle for cpu_bound.
    corners_ll : [(lat, lon), ...] — at least 3 points
    Returns    : [[(lat, lon), ...], ...]  one list per swath

    F2C v1.x flat API:
      f2c.LinearRing, f2c.Point, f2c.Cell
      f2c.SG_BruteForce
      sg.generateSwaths(angle_rad, tool_width, cell)  # pass Cell, get Swaths back
      swaths.size(), swaths.get(i) -> Swath
      swath.getPath() -> LineString, .size(), .getGeometry(j) -> Point
    """
    import math
    import fields2cover as f2c

    lat0, lon0 = corners_ll[0]

    # ── field boundary in local metres ───────────────────────────────────────
    ring = f2c.LinearRing()
    for lat, lon in corners_ll:
        x, y = _f2c_latlon_to_xy(lat, lon, lat0, lon0)
        ring.addPoint(f2c.Point(x, y, 0))
    # close
    x0, y0 = _f2c_latlon_to_xy(corners_ll[0][0], corners_ll[0][1], lat0, lon0)
    ring.addPoint(f2c.Point(x0, y0, 0))

    cell = f2c.Cell()
    cell.addRing(ring)

    # ── swath generation ─────────────────────────────────────────────────────
    # Pass Cell directly — passing Cells gives a SwathsByCells with a different API.
    angle_rad = math.radians(angle_deg % 180)
    sg        = f2c.SG_BruteForce()
    swaths    = sg.generateSwaths(angle_rad, tool_width, cell)

    # ── project back to lat/lon ──────────────────────────────────────────────
    result = []
    for i in range(swaths.size()):
        path   = swaths.get(i).getPath()
        pts_ll = []
        for j in range(path.size()):
            pt = path.getGeometry(j)
            pts_ll.append(_f2c_xy_to_latlon(pt.getX(), pt.getY(), lat0, lon0))
        if len(pts_ll) >= 2:
            result.append(pts_ll)
    return result


# ── ROS node ──────────────────────────────────────────────────────────────────

class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')

        self.cmd_vel_publisher       = self.create_publisher(Twist,  'cmd_vel',       1)
        self.esp_enable_publisher    = self.create_publisher(Empty,  'esp/enable',    1)
        self.esp_disable_publisher   = self.create_publisher(Empty,  'esp/disable',   1)
        self.esp_reset_publisher     = self.create_publisher(Empty,  'esp/reset',     1)
        self.esp_restart_publisher   = self.create_publisher(Empty,  'esp/restart',   1)
        self.esp_configure_publisher = self.create_publisher(Empty,  'esp/configure', 1)
        self.estop_publisher         = self.create_publisher(Bool,   'estop/soft',    SAFETY_QOS)

        _SENSOR_QOS = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(NavSatFix,     '/gnss/fix',           self.store_gps,       _SENSOR_QOS)
        self.create_subscription(BatteryState, 'battery_state',       self.store_battery,               1)
        self.create_subscription(Bool,         'bumper/front_top',    self.update_bumper_front_top,    SAFETY_QOS)
        self.create_subscription(Bool,         'bumper/front_bottom', self.update_bumper_front_bottom, SAFETY_QOS)
        self.create_subscription(Bool,         'bumper/back',         self.update_bumper_back,         SAFETY_QOS)
        self.create_subscription(Bool,         'estop/front',         self.update_estop_front,         SAFETY_QOS)
        self.create_subscription(Bool,         'estop/back',          self.update_estop_back,          SAFETY_QOS)
        self.create_subscription(Odometry,     '/odometry/global',
                                 lambda m: setattr(self, 'latest_odom', m), 10)
        self.create_subscription(String,       '/current_node',
                                 lambda m: setattr(self, 'topo_current', m.data), 10)

        self._topo_doc:  dict            = {}
        self.topo_nodes: dict[str, dict] = _demo_nodes()
        self.topo_demo:  bool            = True
        self.create_subscription(String, '/topological_map_2', self._on_topo_map, TMAP_QOS)
        self._topo_map_pub = self.create_publisher(String, '/topological_map_2', TMAP_QOS)

        if _TOPO_SRV_OK:
            self._write_map_cli  = self.create_client(WriteTopologicalMap,
                '/topological_map_manager2/write_topological_map')
            self._switch_map_cli = self.create_client(WriteTopologicalMap,
                '/topological_map_manager2/switch_topological_map')

        if _ACTION_OK:
            self._nav_ac = ActionClient(self, GotoNode, 'topological_navigation')

        self.latest_odom:    Optional[Odometry]     = None
        self.latest_gps:     Optional[NavSatFix]      = None
        self.latest_battery: Optional[BatteryState] = None

        self.bumper_front_top_active    = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active         = False
        self.soft_estop_active          = False
        self.estop_front_active         = False
        self.estop_back_active          = False
        self.linear_velocity            = 0.0
        self.angular_velocity           = 0.0

        self.topo_selected:   Optional[str] = None
        self.topo_current:    str           = '—'
        self.topo_nav_status: str           = 'Idle'
        self.topo_navigating: bool          = False
        self._nav_goal_handle               = None
        self.drop_status:     str           = ''

        self._track_timer:   Optional[object] = None
        self._track_counter: int              = 0
        self._track_prefix:  str              = ''
        self._track_row_id:  Optional[int]    = None
        self._track_is_row:  bool             = False
        self._track_first:   bool             = True
        self.track_status:   str              = ''

        self._f2c_swaths:     list = []
        self._f2c_row_start:  int  = 1
        self._f2c_tool_width: float = 1.2
        self._f2c_angle_deg:  float = 0.0

        @ui.page('/')
        def page():
            self.content()

    # ── map callback ──────────────────────────────────────────────────────────

    def _on_topo_map(self, msg: String) -> None:
        try:
            nodes, doc      = _parse_topo_json(msg.data)
            self.topo_nodes = nodes
            self._topo_doc  = doc
            self.topo_demo  = False
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /topological_map_2: {e}')

    # ── nav actions ───────────────────────────────────────────────────────────

    def send_nav_goal(self, target: str) -> None:
        if not _ACTION_OK:
            self.topo_nav_status = 'action unavailable (import failed)'; return
        self.topo_nav_status = f'connecting → {target}…'
        self.topo_navigating = True
        import threading
        def _send():
            ready = self._nav_ac.wait_for_server(timeout_sec=5.0)
            if not ready:
                self.topo_nav_status = 'action server not ready (5s timeout)'
                self.topo_navigating = False
                return
            goal = GotoNode.Goal()
            goal.target = target
            self.topo_nav_status = f'→ {target}'
            future = self._nav_ac.send_goal_async(goal, feedback_callback=self._nav_feedback)
            future.add_done_callback(self._nav_accepted)
        threading.Thread(target=_send, daemon=True).start()

    def _nav_accepted(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.topo_nav_status = 'goal rejected'; self.topo_navigating = False; return
        self._nav_goal_handle = gh
        gh.get_result_async().add_done_callback(self._nav_result)

    def _nav_feedback(self, feedback_msg) -> None:
        fb  = feedback_msg.feedback
        loc = getattr(fb, 'current_node', None) or getattr(fb, 'status', '…')
        self.topo_nav_status = f'en route · {loc}'

    def _nav_result(self, future) -> None:
        success = getattr(future.result().result, 'success', True)
        self.topo_nav_status = 'arrived' if success else 'failed'
        self.topo_navigating = False
        self._nav_goal_handle = None

    def cancel_nav_goal(self) -> None:
        if self._nav_goal_handle:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self.topo_nav_status = 'cancelled'
        self.topo_navigating = False

    # ── node dropping ─────────────────────────────────────────────────────────

    def drop_topo_node(self, name: str, row_id: Optional[int],
                       row_role: Optional[str]) -> None:
        name = re.sub(r'[^A-Z0-9_]', '', name.strip().upper().replace(' ', '_'))
        if not name:
            self.drop_status = 'ERROR: node name required'; return
        if not _NAME_RE.match(name):
            self.drop_status = f'ERROR: invalid name "{name}"'; return
        if name in self.topo_nodes:
            self.drop_status = f'ERROR: {name} already exists'; return
        if self.latest_odom is None:
            self.drop_status = 'ERROR: no odometry'; return
        if not self._topo_doc:
            self.drop_status = 'ERROR: map not loaded'; return

        x = round(self.latest_odom.pose.pose.position.x, 3)
        y = round(self.latest_odom.pose.pose.position.y, 3)
        connect_to = (self.topo_current
                      if self.topo_current not in ('—', 'none', 'None', '', None) else None)
        map_name  = self._topo_doc.get('name', 'mixed_test_map')
        nav_frame = self._topo_doc.get('transformation', {}).get('topo_frame_id', 'map')
        is_row    = row_id is not None

        if is_row: edge_action, xy_tol, yaw_tol, vert_r = _ROW_ACTION, 0.1, 0.05, 0.5
        else:      edge_action, xy_tol, yaw_tol, vert_r = _NAV_ACTION, 0.3, 0.1,  1.0

        gps = self.latest_gps
        gps_meta: dict = {}
        if gps is not None and gps.status.status >= 0:
            gps_meta = {'gps_lat': round(gps.latitude, 7), 'gps_lon': round(gps.longitude, 7),
                        'gps_fix_type': int(gps.status.status),
                        'gps_hdop': None}

        row_meta: dict = {'row_id': row_id, 'row_role': row_role or 'entry'} if is_row else {}
        timestamp = datetime.now(timezone.utc).strftime('%d-%m-%Y_%H-%M-%S')
        node_meta_disk = {'map': map_name, 'node': name, 'pointset': map_name}
        node_meta_ui   = {**node_meta_disk, 'dropped_by': 'webui',
                          'timestamp': timestamp, **gps_meta, **row_meta}
        node_properties_disk = {'xy_goal_tolerance': xy_tol, 'yaw_goal_tolerance': yaw_tol,
                                 'dropped_by': 'webui', 'timestamp': timestamp,
                                 **gps_meta, **row_meta}

        def _make_node_dict(meta: dict, props: dict) -> dict:
            return {'meta': meta, 'node': {
                'edges': ([{'action': edge_action, 'edge_id': f'{name}_{connect_to}',
                             'node': connect_to}] if connect_to else []),
                'name': name, 'nav_frame': nav_frame,
                'pose': {'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'position':    {'x': x,   'y': y,   'z': 0.0}},
                'properties': props,
                'verts': [{'x': -vert_r, 'y': -vert_r}, {'x': vert_r, 'y': -vert_r},
                           {'x':  vert_r, 'y':  vert_r}, {'x': -vert_r, 'y':  vert_r}],
            }}

        new_entry = _make_node_dict(node_meta_ui, {'xy_goal_tolerance': xy_tol,
                                                    'yaw_goal_tolerance': yaw_tol})
        new_nodes = dict(self.topo_nodes)
        new_nodes[name] = {'x': x, 'y': y,
                           'edges': [connect_to] if connect_to else [],
                           'meta': node_meta_ui}
        if connect_to:
            if connect_to in new_nodes:
                rev = list(new_nodes[connect_to]['edges'])
                if name not in rev: rev.append(name)
                new_nodes[connect_to] = {**new_nodes[connect_to], 'edges': rev}
            for entry in self._topo_doc.get('nodes', []):
                n = entry.get('node', {})
                if n.get('name') == connect_to:
                    n.setdefault('edges', []).append(
                        {'action': edge_action, 'edge_id': f'{connect_to}_{name}', 'node': name})
                    break

        # Do NOT append to self._topo_doc here — _publish_and_persist reads the
        # file as the single source of truth and syncs _topo_doc after writing.
        # Appending here AND in the thread was the cause of duplicate node names.
        self.topo_nodes = new_nodes

        conn_str = f' → {connect_to}' if connect_to else ''
        gps_str  = (f' [{gps_meta["gps_lat"]:.5f},{gps_meta["gps_lon"]:.5f}]'
                    if gps_meta else '')
        row_str  = f' row={row_id}/{row_role}' if is_row else ''
        self.drop_status = f'{name}{conn_str} at ({x}, {y}){row_str}{gps_str} — writing…'

        def _publish_and_persist():
            try:
                import copy, yaml as _yaml
                map_file      = f'/workspace/maps/{map_name}'
                installed_src = ('/workspace/install/topological_navigation/share/'
                                 'topological_navigation/config/mixed_actions_map.yaml')

                # Read the on-disk file as the authoritative base — never self._topo_doc,
                # which may already contain in-memory-only state from a previous drop.
                if os.path.exists(map_file):
                    with open(map_file) as f: file_doc = _yaml.safe_load(f)
                elif os.path.exists(installed_src):
                    with open(installed_src) as f: file_doc = _yaml.safe_load(f)
                    self.get_logger().info('Seeding from installed source')
                else:
                    file_doc = copy.deepcopy(self._topo_doc)
                    self.get_logger().warn('No YAML source — JSON fallback')

                # Guard: skip if this node was already written (e.g. rapid double-drop)
                existing_names = {e.get('node', {}).get('name')
                                  for e in file_doc.get('nodes', [])}
                if name in existing_names:
                    self.get_logger().warn(f'Node {name} already in file — skipping write')
                    return

                file_doc.setdefault('nodes', []).append(
                    _make_node_dict(node_meta_disk, node_properties_disk))
                if connect_to:
                    for entry in file_doc.get('nodes', []):
                        n = entry.get('node', {})
                        if n.get('name') == connect_to:
                            n.setdefault('edges', []).append(
                                {'action': edge_action,
                                 'edge_id': f'{connect_to}_{name}', 'node': name})
                            break
                with open(map_file, 'w') as f:
                    _yaml.dump(file_doc, f, default_flow_style=False,
                               allow_unicode=True, sort_keys=False)

                # Sync _topo_doc to the file we just wrote so the next drop reads
                # a consistent base — this is the only place _topo_doc is updated.
                self._topo_doc = file_doc

                self.drop_status = (f'{name}{conn_str} at ({x}, {y})'
                                    f'{row_str}{gps_str} — reloading…')

                def _call(client, req, timeout=5.0):
                    ev = threading.Event(); res = [None]
                    def _cb(f): res[0] = f.result(); ev.set()
                    client.call_async(req).add_done_callback(_cb)
                    ev.wait(timeout=timeout); return res[0]

                if _TOPO_SRV_OK:
                    sw = WriteTopologicalMap.Request()
                    sw.filename = f'/workspace/maps/{map_name}'; sw.no_alias = True
                    sr = _call(self._switch_map_cli, sw)
                    if sr and sr.success:
                        self.drop_status = (f'{name}{conn_str} at ({x},{y})'
                                            f'{row_str}{gps_str} — live')
                    else:
                        msg = String(); msg.data = json.dumps(self._topo_doc, ensure_ascii=False)
                        self._topo_map_pub.publish(msg)
                        err = sr.message if sr else 'timeout'
                        self.drop_status = f'{name}{conn_str} saved (switch failed: {err})'
                        self.get_logger().warn(f'switch_topological_map failed ({err})')
                else:
                    msg = String(); msg.data = json.dumps(self._topo_doc, ensure_ascii=False)
                    self._topo_map_pub.publish(msg)
                    self.drop_status = (f'{name}{conn_str} at ({x},{y})'
                                        f'{row_str}{gps_str} — live (no srv)')
                self.get_logger().info(
                    f'Node dropped: {name} at ({x:.3f},{y:.3f}){conn_str}{row_str}{gps_str}')
            except Exception as e:
                self.drop_status = f'ERROR: {e}'
                self.get_logger().error(f'drop_topo_node failed: {e}')

        threading.Thread(target=_publish_and_persist, daemon=True).start()

    # ── track mode ───────────────────────────────────────────────────────────

    def start_track(self, prefix: str, interval: float,
                    row_id: Optional[int], row_role: Optional[str]) -> None:
        prefix = re.sub(r'[^A-Z0-9_]', '', prefix.strip().upper().replace(' ', '_'))
        if not prefix:
            self.track_status = 'ERROR: prefix required'; return
        if self._track_timer is not None:
            self.track_status = 'ERROR: already running'; return
        existing = [n for n in self.topo_nodes
                    if n.startswith(prefix + '_') and n[len(prefix)+1:].isdigit()]
        self._track_counter = (max(int(n[len(prefix)+1:]) for n in existing)
                               if existing else 0)
        self._track_prefix = prefix
        self._track_row_id = row_id
        self._track_is_row = row_id is not None
        self._track_first  = True

        def _drop() -> None:
            self._track_counter += 1
            node_name = f'{prefix}_{self._track_counter}'
            if self._track_is_row:
                role = 'entry' if self._track_first else 'middle'
                self._track_first = False
            else:
                role = row_role
            self.drop_topo_node(node_name, row_id, role)
            self.track_status = f'recording  {node_name}  (#{self._track_counter})'

        _drop()
        self._track_timer = self.create_timer(interval, _drop)

    def stop_track(self) -> None:
        if self._track_timer is not None:
            self._track_timer.cancel(); self._track_timer = None
        if self._track_is_row and self._track_counter > 0:
            last_name = f'{self._track_prefix}_{self._track_counter}'
            self._patch_node_role(last_name, 'exit')
            self.track_status = (f'stopped — {last_name} marked exit'
                                 f'  (#{self._track_counter} nodes)')
        else:
            self.track_status = f'stopped at #{self._track_counter}'
        self._track_counter = 0; self._track_prefix = ''
        self._track_row_id  = None; self._track_is_row = False; self._track_first = True

    def _patch_node_role(self, node_name: str, role: str) -> None:
        import yaml as _yaml
        if node_name in self.topo_nodes:
            nd = dict(self.topo_nodes[node_name])
            nd['meta'] = {**nd.get('meta', {}), 'row_role': role}
            new_nodes = dict(self.topo_nodes); new_nodes[node_name] = nd
            self.topo_nodes = new_nodes
        for entry in self._topo_doc.get('nodes', []):
            n = entry.get('node', {})
            if n.get('name') == node_name:
                entry.get('meta', {})['row_role'] = role
                n.get('properties', {})['row_role'] = role; break
        map_name = self._topo_doc.get('name', 'mixed_test_map')
        map_file = f'/workspace/maps/{map_name}'
        def _write():
            try:
                if not os.path.exists(map_file): return
                with open(map_file) as f: doc = _yaml.safe_load(f)
                for entry in doc.get('nodes', []):
                    n = entry.get('node', {})
                    if n.get('name') == node_name:
                        entry.get('meta', {})['row_role'] = role
                        n.get('properties', {})['row_role'] = role; break
                with open(map_file, 'w') as f:
                    _yaml.dump(doc, f, default_flow_style=False,
                               allow_unicode=True, sort_keys=False)
            except Exception as e:
                self.get_logger().error(f'_patch_node_role failed: {e}')
        threading.Thread(target=_write, daemon=True).start()

    # ── UI shell ──────────────────────────────────────────────────────────────

    def content(self) -> None:
        ui.add_head_html(_GLOBAL_CSS)
        with ui.tabs().classes('w-full') as tabs:
            tab_nav     = ui.tab('Nav',     icon='route')
            tab_mission = ui.tab('Mission', icon='checklist')
            tab_system  = ui.tab('System',  icon='settings')
        with ui.tab_panels(tabs, value=tab_nav).classes('w-full'):
            with ui.tab_panel(tab_nav):     self._nav_content()
            with ui.tab_panel(tab_mission): self._mission_content()
            with ui.tab_panel(tab_system):  self._system_content()

    # ── Nav tab ───────────────────────────────────────────────────────────────
    #
    #  ┌─────────────────────────────────────────────────────┐  ┌──────────┐
    #  │  [Joystick | E-Stop+pose]  │  Node Map (flex)       │  │ Sidebar  │
    #  ├────────────────────────────┴───────────────────────-┤  │ full ht  │
    #  │  Track (flex-1)   │   Drop Node (flex-1)            │  │          │
    #  └─────────────────────────────────────────────────────┘  └──────────┘

    def _nav_content(self) -> None:

        with ui.row().classes('w-full gap-3 items-stretch'):

            # ── Main column ───────────────────────────────────────────────────
            with ui.column().classes('flex-1 gap-3').style('min-width:0'):

                # Row 1 — joystick card + node map
                with ui.row().classes('w-full gap-3 items-stretch'):

                    # Joystick card: joy on left, e-stop+pose on right
                    with ui.card().style('padding:16px 20px;flex-shrink:0'):
                        ui.html('<div class="sec-label mb-3">Joystick</div>')
                        with ui.row().classes('items-center gap-6'):
                            ui.joystick(
                                color='#1a7f37', size=130,
                                on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                                on_end=lambda _: self.send_speed(0.0, 0.0),
                            )
                            with ui.column().classes('gap-3 items-center'):
                                def update_estop(e: ClickEventArguments) -> None:
                                    assert isinstance(e.sender, ui.button)
                                    self.toggle_estop()
                                    if self.soft_estop_active:
                                        e.sender.props('color=negative')
                                        e.sender.text = 'STOPPED'
                                    else:
                                        e.sender.props('color=primary')
                                        e.sender.text = 'E-Stop'
                                ui.button('E-Stop', on_click=update_estop).props(
                                    'color=primary outline no-caps').classes('estop-btn')
                                pose_lbl = ui.label('—').classes('text-xs font-mono').style(
                                    'color:#57606a;text-align:center;'
                                    'max-width:130px;white-space:pre-line')

                    # Node map
                    with ui.card().classes('flex-1').style('padding:12px;min-width:0'):
                        ui.html('<div class="sec-label mb-2">Node Map</div>')
                        map_html = ui.html(_build_svg(self.topo_nodes, None, None))

                # Row 2 — Track + Drop Node
                with ui.row().classes('w-full gap-3 items-start'):

                    # Track
                    with ui.card().classes('flex-1').style('padding:12px 14px'):
                        with ui.row().classes('items-baseline gap-2 mb-2'):
                            ui.label('Track').classes('font-semibold')
                            ui.label('auto-drop every N s').classes('text-xs').style(
                                'color:#8c959f')
                        with ui.row().classes('items-center gap-2 w-full'):
                            track_prefix = ui.input(
                                placeholder='Prefix e.g. ROW_A', label='Prefix',
                            ).classes('flex-1')
                            track_interval = ui.number(
                                label='s', value=5, min=2, max=30, step=1, precision=0,
                            ).classes('w-16')
                        with ui.row().classes('items-center gap-2 w-full mt-1'):
                            track_row_id = ui.number(
                                label='Row ID', placeholder='blank=standard',
                                min=1, step=1, precision=0,
                            ).classes('w-28')
                            track_row_role = ui.toggle(
                                {'entry': 'Entry', 'exit': 'Exit'}, value='entry',
                            ).props('dense')
                            track_row_hint = ui.label('').classes('text-xs font-mono').style(
                                'color:#8c959f')
                        with ui.row().classes('items-center gap-2 mt-2'):
                            track_start_btn = ui.button(
                                'Start',
                                on_click=lambda: self.start_track(
                                    track_prefix.value,
                                    float(track_interval.value or 5),
                                    int(track_row_id.value) if track_row_id.value else None,
                                    track_row_role.value,
                                ),
                            ).props('color=positive no-caps dense')
                            track_stop_btn = ui.button(
                                'Stop', on_click=self.stop_track,
                            ).props('color=negative no-caps dense')
                            track_status_lbl = ui.label('').classes(
                                'text-xs font-mono ml-1').style('color:#57606a')

                    # Drop Node
                    with ui.card().classes('flex-1').style('padding:12px 14px'):
                        with ui.row().classes('items-baseline gap-2 mb-2'):
                            ui.label('Drop Node').classes('font-semibold')
                            ui.label('pins at current pose').classes('text-xs').style(
                                'color:#8c959f')
                        with ui.row().classes('items-center gap-2 w-full'):
                            name_input = ui.input(
                                placeholder='e.g. ROW_D_IN', label='Name',
                            ).classes('flex-1')
                        with ui.row().classes('items-center gap-2 w-full mt-1'):
                            row_id_input = ui.number(
                                label='Row ID', placeholder='blank=standard',
                                min=1, step=1, precision=0,
                            ).classes('w-28')
                            row_role_toggle = ui.toggle(
                                {'entry': 'Entry', 'exit': 'Exit'}, value='entry',
                            ).props('dense')
                            row_hint = ui.label('').classes('text-xs font-mono').style(
                                'color:#8c959f')
                        with ui.row().classes('items-center gap-2 mt-2'):
                            cur_drop_lbl = ui.label('').classes('text-xs font-mono').style(
                                'color:#8c959f')
                            ui.button(
                                'Drop',
                                on_click=lambda: self.drop_topo_node(
                                    name_input.value,
                                    int(row_id_input.value) if row_id_input.value else None,
                                    row_role_toggle.value,
                                ),
                            ).classes('ml-auto').props('color=positive no-caps dense')
                        drop_status_lbl = ui.label('').classes('text-xs font-mono mt-1')

            # ── Sidebar ───────────────────────────────────────────────────────
            with ui.card().classes('nav-sidebar').style('padding:14px'):
                ui.html('<div class="sec-label">Current node</div>')
                cur_lbl = ui.label('—').classes('text-sm font-mono font-bold')

                ui.html('<div class="sec-label mt-3">Destination</div>')
                sel_lbl = ui.label('—').classes('text-sm font-mono').style('color:#8c959f')

                ui.html('<div class="sec-label mt-3">Status</div>')
                stat_lbl = ui.label('idle').classes('text-xs font-mono').style('color:#57606a')

                ui.separator().classes('my-2')

                go_btn   = ui.button('Go',     color='positive').classes('w-full').props('no-caps')
                stop_btn = ui.button('Cancel', color='negative').classes('w-full').props(
                    'no-caps flat')

                ui.html('<div class="sec-label mt-3">Nodes</div>')
                with ui.scroll_area().style('flex:1;min-height:0'):
                    node_col = ui.column().style('gap:1px;width:100%')

        # ── wiring ────────────────────────────────────────────────────────────

        go_btn.on_click(
            lambda: self.send_nav_goal(self.topo_selected) if self.topo_selected else None)
        stop_btn.on_click(self.cancel_nav_goal)

        def on_node_clicked(e) -> None:
            n = (e.args or {}).get('node')
            if n and n in self.topo_nodes:
                self.topo_selected = n
        ui.on('topo_node_clicked', on_node_clicked)

        def inject_click_js() -> None:
            ui.run_javascript("""
                setTimeout(() => {
                    document.querySelectorAll('.topo-node').forEach(el => {
                        el.onclick = function() {
                            emitEvent('topo_node_clicked', {node: this.dataset.node});
                        };
                    });
                }, 150);
            """)

        _prev: dict = {}

        def refresh_nav() -> None:
            # Pose / GPS label
            odom = self.latest_odom
            gps  = self.latest_gps
            if odom is not None:
                px, py  = odom.pose.pose.position.x, odom.pose.pose.position.y
                gps_str = (f'\n{gps.latitude:.5f}\n{gps.longitude:.5f}'
                           if gps and gps.status.status >= 0 else '')
                pose_lbl.set_text(f'({px:.2f}, {py:.2f}){gps_str}')
            else:
                pose_lbl.set_text('no odom')

            # Drop hints
            cur = self.topo_current
            if cur and cur != '—':
                cur_drop_lbl.set_text(f'→ {cur}'); cur_drop_lbl.style('color:#1a7f37')
            else:
                cur_drop_lbl.set_text('no current node'); cur_drop_lbl.style('color:#8c959f')
            row_hint.set_text(_ROW_ACTION if row_id_input.value else _NAV_ACTION)
            row_hint.style('color:#0969da' if row_id_input.value else 'color:#8c959f')
            drop_status_lbl.set_text(self.drop_status)
            drop_status_lbl.style(
                'color:#cf222e' if self.drop_status.startswith('ERROR') else 'color:#1a7f37')

            # Track hints
            running = self._track_timer is not None
            track_start_btn.set_enabled(not running)
            track_stop_btn.set_enabled(running)
            if track_row_id.value:
                track_row_hint.set_text('entry→middle→exit auto')
                track_row_hint.style('color:#0969da')
                track_row_role.set_visibility(False)
            else:
                track_row_hint.set_text(_NAV_ACTION)
                track_row_hint.style('color:#8c959f')
                track_row_role.set_visibility(True)
            track_status_lbl.set_text(self.track_status)
            track_status_lbl.style(
                'color:#cf222e' if self.track_status.startswith('ERROR') else
                'color:#1a7f37' if running else 'color:#57606a')

            # Map + sidebar — only repaint when something changed
            snap = {'sel': self.topo_selected, 'cur': self.topo_current,
                    'stat': self.topo_nav_status, 'nav': self.topo_navigating,
                    'nodes': id(self.topo_nodes)}
            changed = {k for k, v in snap.items() if _prev.get(k) != v}
            if not changed:
                return
            _prev.update(snap)

            if changed & {'sel', 'cur', 'nodes'}:
                map_html.set_content(
                    _build_svg(self.topo_nodes, self.topo_selected, self.topo_current))
                inject_click_js()
                node_col.clear()
                with node_col:
                    for nname in sorted(self.topo_nodes):
                        nd       = self.topo_nodes[nname]
                        is_row   = nd.get('meta', {}).get('row_id') is not None
                        rid_v    = nd.get('meta', {}).get('row_id', '')
                        rrole_v  = nd.get('meta', {}).get('row_role', '')
                        glat     = nd.get('meta', {}).get('gps_lat', '')
                        glon     = nd.get('meta', {}).get('gps_lon', '')
                        title    = (f'Row {rid_v} {rrole_v} | {glat} {glon}'.strip()
                                    if is_row else f'{glat} {glon}'.strip())
                        cls      = ('node-item'
                                    + (' sel' if nname == self.topo_selected else '')
                                    + (' row' if is_row else ''))
                        n = nname
                        ui.html(
                            f'<div class="{cls}" title="{title}">'
                            f'{nname}{"  · row" if is_row else ""}</div>'
                        ).on('click', lambda _, n=n: setattr(self, 'topo_selected', n))

            cur_lbl.set_text(self.topo_current or '—')
            sel_lbl.set_text(self.topo_selected or '—')
            sel_lbl.style('color:#9a6700' if self.topo_selected else 'color:#8c959f')
            stat_lbl.set_text(self.topo_nav_status)
            stat_lbl.style(
                'color:#cf222e' if 'fail' in self.topo_nav_status else
                'color:#1a7f37' if self.topo_nav_status == 'arrived' else 'color:#57606a')
            can_go = (bool(self.topo_selected) and not self.topo_navigating
                      and not self.soft_estop_active)
            go_btn.set_enabled(can_go)
            stop_btn.set_enabled(self.topo_navigating)

        ui.timer(0.2, refresh_nav)
        inject_click_js()

    # ── Mission tab ───────────────────────────────────────────────────────────
    #
    # Layout:
    #   ┌─────────────────────────────────────────────┬──────────────────────┐
    #   │  Leaflet map (click to draw polygon)        │  Sidebar controls    │
    #   │                                             │  tool_width          │
    #   │                                             │  angle slider        │
    #   │                                             │  row_id_start        │
    #   │                                             │  Plan / Clear        │
    #   │                                             │  status              │
    #   └─────────────────────────────────────────────┴──────────────────────┘
    #   ┌────────────────────────────────────────────────────────────────────┐
    #   │  Mission Queue                                                     │
    #   └────────────────────────────────────────────────────────────────────┘

    def _mission_content(self) -> None:
        # ── State ─────────────────────────────────────────────────────────────
        corners_ll: list[tuple[float, float]] = []   # [(lat, lon), ...]
        swath_layers: list = []                       # leaflet layer handles
        poly_layer:   list = [None]                   # single polygon layer

        # ── Layout ────────────────────────────────────────────────────────────
        with ui.row().classes('w-full gap-3 items-start mb-3'):

            # Map
            with ui.card().classes('flex-1').style('padding:10px;min-width:0'):
                ui.html('<div class="sec-label mb-2">Field boundary — click to draw</div>')
                gps_center = (
                    (self.latest_gps.latitude, self.latest_gps.longitude)
                    if self.latest_gps else (51.5395, -2.4435)
                )
                mission_map = ui.leaflet(center=gps_center, zoom=18).classes('w-full h-96')
                mission_map.tile_layer(
                    url_template='https://server.arcgisonline.com/ArcGIS/rest/services/'
                                 'World_Imagery/MapServer/tile/{z}/{y}/{x}',
                    options={'attribution': 'Esri', 'maxZoom': 20},
                )

            # Sidebar
            with ui.card().style('width:220px;flex-shrink:0;padding:14px'):
                ui.html('<div class="sec-label">Tool width</div>')
                f2c_width = ui.number(
                    value=1.2, min=0.1, max=10.0, step=0.1, precision=2,
                    suffix='m',
                ).classes('w-full')

                ui.html('<div class="sec-label mt-3">Row angle</div>')
                f2c_angle = ui.slider(min=0, max=179, step=1, value=0).classes('w-full')
                angle_lbl = ui.label('0°').classes('text-xs font-mono').style('color:#57606a')
                f2c_angle.on('update:model-value',
                             lambda e: angle_lbl.set_text(f'{int(e.args)}°'))

                ui.html('<div class="sec-label mt-3">First row ID</div>')
                f2c_row_id_start = ui.number(
                    value=1, min=1, step=1, precision=0,
                ).classes('w-full')

                ui.separator().classes('my-3')

                corners_lbl = ui.label('0 corners').classes('text-xs font-mono').style(
                    'color:#57606a')

                plan_btn  = ui.button('Plan Rows').props(
                    'color=positive no-caps').classes('w-full mt-2')
                clear_btn = ui.button('Clear').props(
                    'outline no-caps').classes('w-full mt-1')

                f2c_status = ui.label('').classes('text-xs font-mono mt-2').style(
                    'color:#57606a;word-break:break-word')

        # ── Map click handler ─────────────────────────────────────────────────

        def _redraw_polygon():
            if poly_layer[0] is not None:
                try:
                    poly_layer[0].run_method('remove')
                except Exception:
                    pass
                poly_layer[0] = None
            if len(corners_ll) >= 2:
                latlngs = [[lat, lon] for lat, lon in corners_ll]
                poly_layer[0] = mission_map.generic_layer(
                    'polygon',
                    latlngs,
                    {'color': '#1a7f37', 'fillOpacity': 0.15,
                     'weight': 2, 'dashArray': '6 4'},
                )
            corners_lbl.set_text(
                f'{len(corners_ll)} corner{"s" if len(corners_ll) != 1 else ""}' +
                (' ✓' if len(corners_ll) >= 3 else ' — need 3+'))

        def on_map_click(e):
            lat = e.args['latlng']['lat']
            lon = e.args['latlng']['lng']
            corners_ll.append((lat, lon))
            mission_map.marker(latlng=(lat, lon))
            _redraw_polygon()

        mission_map.on('map-click', on_map_click)

        # ── Clear ─────────────────────────────────────────────────────────────

        def do_clear():
            corners_ll.clear()
            for lyr in swath_layers:
                try: lyr.run_method('remove')
                except Exception: pass
            swath_layers.clear()
            if poly_layer[0] is not None:
                try: poly_layer[0].run_method('remove')
                except Exception: pass
                poly_layer[0] = None
            corners_lbl.set_text('0 corners')
            f2c_status.set_text('')
            # Rebuild map to drop markers (leaflet markers have no easy bulk-remove)
            mission_map.set_center(mission_map.center)

        clear_btn.on_click(do_clear)

        # ── Plan ──────────────────────────────────────────────────────────────

        async def do_plan():
            if len(corners_ll) < 3:
                f2c_status.set_text('Need at least 3 corners')
                f2c_status.style('color:#cf222e')
                return

            plan_btn.set_enabled(False)
            f2c_status.set_text('Running F2C…')
            f2c_status.style('color:#57606a')

            width     = float(f2c_width.value or 1.2)
            angle_deg = float(f2c_angle.value or 0)
            row_start = int(f2c_row_id_start.value or 1)

            try:
                from nicegui import run as ng_run
                swaths = await ng_run.cpu_bound(
                    _run_f2c, list(corners_ll), width, angle_deg)
            except Exception as exc:
                f2c_status.set_text(f'ERROR: {exc}')
                f2c_status.style('color:#cf222e')
                plan_btn.set_enabled(True)
                return

            # Clear old swaths
            for lyr in swath_layers:
                try: lyr.run_method('remove')
                except Exception: pass
            swath_layers.clear()

            # Draw new swaths
            for i, pts in enumerate(swaths):
                latlngs = [[lat, lon] for lat, lon in pts]
                lyr = mission_map.generic_layer(
                    'polyline', latlngs,
                    {'color': '#0969da', 'weight': 2, 'opacity': 0.85},
                )
                swath_layers.append(lyr)

            # Store on self for future ROS publishing
            self._f2c_swaths     = swaths
            self._f2c_row_start  = row_start
            self._f2c_tool_width = width
            self._f2c_angle_deg  = angle_deg

            f2c_status.set_text(
                f'{len(swaths)} rows · {width}m wide · {angle_deg:.0f}°')
            f2c_status.style('color:#1a7f37')
            plan_btn.set_enabled(True)

        plan_btn.on_click(do_plan)

        with ui.card().classes('w-full'):
            with ui.row().classes('items-baseline gap-2 mb-3'):
                ui.label('Mission Queue').classes('font-semibold')
                ui.label('select rows to run today').classes('text-xs').style('color:#8c959f')
            with ui.row().classes('w-full gap-4 items-start'):
                with ui.card().classes('flex-1').style('background:#f6f8fa;padding:10px'):
                    ui.html('<div class="sec-label mb-2">Available rows</div>')
                    available_col = ui.column().style('gap:2px;width:100%')
                with ui.card().classes('flex-1').style('background:#f6f8fa;padding:10px'):
                    ui.html('<div class="sec-label mb-2">Today\'s queue</div>')
                    queue_col = ui.column().style('gap:2px;width:100%')
                    queue_lbl = ui.label('Empty — add rows from the left').classes(
                        'text-xs').style('color:#8c959f')
            mission_queue: list = []
            def _render_queue():
                queue_col.clear()
                if not mission_queue: queue_lbl.set_visibility(True); return
                queue_lbl.set_visibility(False)
                with queue_col:
                    for i, rid in enumerate(mission_queue):
                        r = i
                        with ui.row().classes('items-center gap-1 w-full'):
                            ui.label(f'Row {rid}').classes('text-sm font-mono flex-1')
                            ui.button('↑', on_click=lambda _, r=r: _move(r, -1)).props(
                                'flat dense').classes('text-xs').style('color:#57606a').set_enabled(i > 0)
                            ui.button('↓', on_click=lambda _, r=r: _move(r, 1)).props(
                                'flat dense').classes('text-xs').style('color:#57606a').set_enabled(i < len(mission_queue)-1)
                            ui.button('✕', on_click=lambda _, r=r: _remove(r)).props(
                                'flat dense').classes('text-xs').style('color:#cf222e')
            def _move(idx, d):
                ni = idx+d
                if 0 <= ni < len(mission_queue):
                    mission_queue[idx], mission_queue[ni] = mission_queue[ni], mission_queue[idx]
                _render_queue()
            def _remove(idx): mission_queue.pop(idx); _render_queue()
            def _add_row(row_id):
                if row_id not in mission_queue: mission_queue.append(row_id)
                _render_queue()
            mission_status = ui.label('').classes('text-xs font-mono mt-3').style('color:#57606a')
            with ui.row().classes('gap-2 mt-3'):
                ui.button('Run Mission', on_click=lambda: self._run_mission(
                    mission_queue, mission_status)).props('color=positive no-caps')
                ui.button('Cancel', on_click=self.cancel_nav_goal).props('color=negative no-caps flat')
            _mprev = [None]
            def refresh_mission():
                if id(self.topo_nodes) == _mprev[0]: return
                _mprev[0] = id(self.topo_nodes)
                rows: dict[int, str] = {}
                for nname, nd in self.topo_nodes.items():
                    meta = nd.get('meta', {})
                    rid  = meta.get('row_id')
                    if rid is not None and meta.get('row_role', '') == 'entry':
                        rows[int(rid)] = nname
                available_col.clear()
                if not rows:
                    with available_col:
                        ui.label('No rows in map yet').classes('text-xs').style('color:#8c959f')
                else:
                    with available_col:
                        for rid in sorted(rows):
                            r = rid
                            with ui.row().classes('items-center gap-2 w-full'):
                                ui.label(f'Row {rid}').classes('text-sm font-mono flex-1')
                                ui.label(rows[rid]).classes('text-xs font-mono').style('color:#8c959f')
                                ui.button('Add →', on_click=lambda _, r=r: _add_row(r)).props(
                                    'color=primary outline no-caps dense')
            ui.timer(1.0, refresh_mission)

    def _run_mission(self, queue: list, status_lbl) -> None:
        if not queue:
            status_lbl.set_text('ERROR: queue is empty'); status_lbl.style('color:#cf222e'); return
        status_lbl.set_text(f'Mission executor not yet implemented — queue: rows {queue}')
        status_lbl.style('color:#9a6700')

    # ── System tab ────────────────────────────────────────────────────────────

    def _system_content(self) -> None:
        with ui.row().classes('items-stretch w-full gap-3'):
            with ui.card().classes('flex-1'):
                ui.label('Telemetry').classes('font-semibold mb-2')
                ui.html('<div class="sec-label">Linear velocity</div>')
                ui.slider(min=-1, max=1, step=0.05, value=0).props(
                    'readonly selection-color=transparent color=green').bind_value(self, 'linear_velocity')
                ui.html('<div class="sec-label mt-2">Angular velocity</div>')
                ui.slider(min=-1, max=1, step=0.05, value=0).props(
                    'readonly selection-color=transparent color=green').bind_value(self, 'angular_velocity')
                ui.html('<div class="sec-label mt-3">Battery</div>')
                ui.label().classes('text-sm').bind_text_from(self, 'latest_battery',
                    lambda msg: (f'{msg.percentage*100:.1f}%  {msg.voltage:.1f} V'
                                 if msg is not None else '—'))
            with ui.card().classes('flex-1'):
                ui.label('Safety').classes('font-semibold mb-2')
                ui.html('<div class="sec-label">Bumpers</div>')
                for attr, label in [('bumper_front_top_active', 'Front top'),
                                    ('bumper_front_bottom_active', 'Front bottom'),
                                    ('bumper_back_active', 'Rear')]:
                    with ui.row().classes('items-center gap-0'):
                        dot = ui.html('<span class="dot-off"></span>')
                        ui.label(label).classes('text-sm')
                    def _mk(d=dot, a=attr):
                        def _u(): d.set_content(f'<span class="dot-{"warn" if getattr(self,a) else "ok"}"></span>')
                        return _u
                    ui.timer(0.2, _mk())
                ui.html('<div class="sec-label mt-3">E-stops</div>')
                for attr, label in [('estop_front_active', 'Front'), ('estop_back_active', 'Rear')]:
                    with ui.row().classes('items-center gap-0'):
                        dot = ui.html('<span class="dot-off"></span>')
                        ui.label(label).classes('text-sm')
                    def _mk2(d=dot, a=attr):
                        def _u(): d.set_content(f'<span class="dot-{"warn" if getattr(self,a) else "off"}"></span>')
                        return _u
                    ui.timer(0.2, _mk2())
        with ui.card().classes('w-full mt-3'):
            ui.label('ESP').classes('font-semibold mb-2')
            with ui.row().classes('gap-2 flex-wrap'):
                ui.button('Enable',    on_click=lambda: self.esp_enable_publisher.publish(Empty())).props('color=positive outline no-caps').classes('px-4')
                ui.button('Disable',   on_click=lambda: self.esp_disable_publisher.publish(Empty())).props('color=negative outline no-caps').classes('px-4')
                ui.button('Reset',     on_click=lambda: self.esp_reset_publisher.publish(Empty())).props('color=warning outline no-caps').classes('px-4')
                ui.button('Restart',   on_click=lambda: self.esp_restart_publisher.publish(Empty())).props('color=primary outline no-caps').classes('px-4')
                ui.button('Configure', on_click=lambda: self.esp_configure_publisher.publish(Empty())).props('outline no-caps').classes('px-4')
        with ui.card().classes('w-full mt-3'):
            ui.label('GPS').classes('font-semibold mb-2')
            leaflet = ui.leaflet(center=(51.5395, -2.4435), zoom=18).classes('w-full h-80')
            marker  = leaflet.marker(latlng=leaflet.center)
            gps_status_lbl = ui.label('—').classes('text-xs font-mono mt-1').style('color:#57606a')
            _FIX_LABELS = {-1: 'NO FIX', 0: 'AUTONOMOUS', 1: 'SBAS',
                            2: 'DGNSS', 4: 'RTK FLOAT', 5: 'RTK FIXED'}
            def update_gps_ui():
                if self.latest_gps is not None:
                    lat, lon = self.latest_gps.latitude, self.latest_gps.longitude
                    leaflet.set_center((lat, lon))
                    marker.move(lat, lon)
                    code = self.latest_gps.status.status
                    cov  = self.latest_gps.position_covariance[0]
                    gps_status_lbl.set_text(
                        f'{_FIX_LABELS.get(code, str(code))}  '
                        f'{lat:.6f}, {lon:.6f}  '
                        f'alt={self.latest_gps.altitude:.1f}m  '
                        f'σ={cov**0.5:.2f}m')
                    col = '#1a7f37' if code == 5 else '#9a6700' if code >= 1 else '#cf222e'
                    gps_status_lbl.style(f'color:{col}')
            ui.timer(2.0, update_gps_ui)
        with ui.card().classes('w-full mt-3'):
            ui.label('Tools').classes('font-semibold mb-2')
            with ui.row().classes('items-center gap-3 flex-wrap'):
                _explorer_proc: list = [None]
                _explorer_lbl = ui.label('').classes('text-xs font-mono').style('color:#57606a')
                def _start_explorer():
                    import subprocess
                    if _explorer_proc[0] is not None and _explorer_proc[0].poll() is None:
                        _explorer_lbl.set_text('already running')
                        return
                    try:
                        _explorer_proc[0] = subprocess.Popen(
                            ['ros2', 'run', 'ros2graph_explorer', 'ros2graph_explorer'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        )
                        _explorer_lbl.set_text(f'started (pid {_explorer_proc[0].pid})')
                        _explorer_lbl.style('color:#1a7f37')
                    except Exception as exc:
                        _explorer_lbl.set_text(f'ERROR: {exc}')
                        _explorer_lbl.style('color:#cf222e')
                ui.button('Start Graph Explorer', on_click=_start_explorer).props(
                    'outline no-caps').classes('px-4')
                ui.html(
                    '<a href="http://localhost:8734/" target="_blank" '
                    'style="font-size:13px;color:var(--blue);text-decoration:none;'
                    'padding:6px 12px;border:1px solid var(--blue);border-radius:4px;'
                    'font-family:\'Courier New\',monospace;">'
                    '↗ Graph Explorer</a>'
                )
                _explorer_lbl

    # ── shared ────────────────────────────────────────────────────────────────

    def toggle_estop(self) -> None:
        self.soft_estop_active = not self.soft_estop_active
        msg = Bool(); msg.data = self.soft_estop_active
        self.estop_publisher.publish(msg)

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x; msg.angular.z = -y
        self.linear_velocity = x; self.angular_velocity = y
        self.cmd_vel_publisher.publish(msg)

    def store_gps(self, msg: NavSatFix) -> None:             self.latest_gps = msg
    def store_battery(self, msg: BatteryState) -> None:   self.latest_battery = msg
    def update_bumper_front_top(self, msg: Bool) -> None:    self.bumper_front_top_active = msg.data
    def update_bumper_front_bottom(self, msg: Bool) -> None: self.bumper_front_bottom_active = msg.data
    def update_bumper_back(self, msg: Bool) -> None:         self.bumper_back_active = msg.data
    def update_estop_front(self, msg: Bool) -> None:         self.estop_front_active = msg.data
    def update_estop_back(self, msg: Bool) -> None:          self.estop_back_active = msg.data


# ── entrypoints ───────────────────────────────────────────────────────────────

def main() -> None:
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=80)
