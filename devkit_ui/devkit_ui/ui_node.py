# pylint: disable=duplicate-code
"""
ui_node.py — Sowbot web cockpit on :80

Tabs
  Navigate  — topo map, node picker, GO/CANCEL, joystick, drop node, track mode (DEFAULT)
  Control   — telemetry, safety, ESP, GPS map

Node drop workflow:
  1. Validate name + pose
  2. Add node to _topo_doc in memory
  3. Write updated YAML to /workspace/maps/mixed_test_map
  4. Call switch_topological_map so manager reloads the file
  5. On switch failure, fall back to publishing JSON directly to the topic.
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
from gps_msgs.msg import GPSFix
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

# Valid node name: letters, digits, underscores only
_NAME_RE = re.compile(r'^[A-Z0-9_]+$')

# ── Global CSS ────────────────────────────────────────────────────────────────

_GLOBAL_CSS = """
<style>
:root {
  --bg:        #f6f8fa;
  --bg-card:   #ffffff;
  --border:    #d0d7de;
  --border-hi: #8c959f;
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

/* Section label */
.sec-label {
  font-size: 10px; font-weight: 600; letter-spacing: 0.08em;
  text-transform: uppercase; color: var(--txt-muted);
  font-family: 'Courier New', monospace; margin-bottom: 2px;
}

/* Indicator dot */
.dot-ok   { display:inline-block; width:7px; height:7px; border-radius:50%;
            background:#1a7f37; margin-right:6px; }
.dot-warn { display:inline-block; width:7px; height:7px; border-radius:50%;
            background:#9a6700; margin-right:6px; }
.dot-off  { display:inline-block; width:7px; height:7px; border-radius:50%;
            background:#d0d7de; margin-right:6px; }

/* Nav node list */
.node-item {
  display: block; padding: 4px 8px; border-radius: 4px;
  font-family: 'Courier New', monospace; font-size: 11px;
  cursor: pointer; border-left: 3px solid transparent;
  color: var(--txt-dim); transition: background 0.1s;
}
.node-item:hover { background: #f6f8fa; color: var(--txt); }
.node-item.sel   { background: #fff8c5; color: var(--amber);
                   border-left-color: var(--amber); }
.node-item.row   { color: var(--blue); border-left-color: #d8e8fd; }
.node-item.sel.row { background: #fff8c5; color: var(--amber);
                     border-left-color: var(--amber); }

/* Map SVG hover */
.topo-node:hover { filter: brightness(0.85); cursor: pointer; }

/* Status pill */
.pill {
  display: inline-block; padding: 1px 8px; border-radius: 999px;
  font-size: 11px; font-family: 'Courier New', monospace; font-weight: 600;
}
.pill-ok   { background: #dafbe1; color: #1a7f37; border: 1px solid #aceebb; }
.pill-warn { background: #fff8c5; color: #9a6700; border: 1px solid #e3b341; }
.pill-off  { background: #f6f8fa; color: #8c959f; border: 1px solid #d0d7de; }
</style>
"""

# ── live map parser ───────────────────────────────────────────────────────────

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
        nodes[name] = {
            'x':     float(pos['x']),
            'y':     float(pos['y']),
            'edges': edges,
            'meta':  meta,
        }
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


# ── SVG map renderer ──────────────────────────────────────────────────────────

_SVG_W, _SVG_H = 720, 340
_MARGIN, _NODE_R = 48, 17


def _build_svg(nodes: dict, selected: Optional[str], current: Optional[str]) -> str:
    if not nodes:
        return (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
                f'style="background:#f6f8fa;border-radius:4px;border:1px solid #d0d7de">'
                f'<text x="{_SVG_W//2}" y="{_SVG_H//2}" text-anchor="middle" '
                f'fill="#8c959f" font-family="Courier New" font-size="13">'
                f'No map loaded</text></svg>')

    xs    = [n['x'] for n in nodes.values()]
    ys    = [n['y'] for n in nodes.values()]
    dx    = (max(xs) - min(xs)) or 1.0
    dy    = (max(ys) - min(ys)) or 1.0
    x_min = min(xs)
    y_min = min(ys)

    def tx(x): return _MARGIN + (x - x_min) / dx * (_SVG_W - 2 * _MARGIN)
    def ty(y): return _SVG_H - _MARGIN - (y - y_min) / dy * (_SVG_H - 2 * _MARGIN)

    parts: list[str] = [
        f'<rect width="{_SVG_W}" height="{_SVG_H}" fill="#f6f8fa" rx="4"/>',
    ]

    # Edges
    drawn: set = set()
    for name, nd in nodes.items():
        for tgt in nd['edges']:
            key = tuple(sorted([name, tgt]))
            if key in drawn or tgt not in nodes:
                continue
            drawn.add(key)
            is_row_edge = (nd.get('meta', {}).get('row_id') is not None
                           or nodes[tgt].get('meta', {}).get('row_id') is not None)
            stroke = '#d8e8fd' if is_row_edge else '#d0d7de'
            sw     = 2 if is_row_edge else 1
            parts.append(
                f'<line x1="{tx(nd["x"]):.1f}" y1="{ty(nd["y"]):.1f}" '
                f'x2="{tx(nodes[tgt]["x"]):.1f}" y2="{ty(nodes[tgt]["y"]):.1f}" '
                f'stroke="{stroke}" stroke-width="{sw}" stroke-linecap="round"/>')

    # Nodes
    for name, nd in nodes.items():
        cx, cy  = tx(nd['x']), ty(nd['y'])
        is_cur  = (name == current)
        is_sel  = (name == selected)
        is_row  = nd.get('meta', {}).get('row_id') is not None
        is_drop = bool(nd.get('meta', {}).get('dropped_by'))

        if is_cur:
            fill, stroke, sw = '#dafbe1', '#1a7f37', 2
        elif is_sel:
            fill, stroke, sw = '#fff8c5', '#9a6700', 2
        elif is_row:
            fill, stroke, sw = '#d8e8fd', '#0969da', 1
        elif is_drop:
            fill, stroke, sw = '#f6f8fa', '#8c959f', 1
        else:
            fill, stroke, sw = '#ffffff', '#d0d7de', 1

        label_col = '#1a7f37' if is_cur else ('#9a6700' if is_sel else
                    ('#0969da' if is_row else '#57606a'))

        if is_cur:
            parts.append(
                f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R + 6}" '
                f'fill="none" stroke="#1a7f37" stroke-width="1" opacity="0.4">'
                f'<animate attributeName="r" values="{_NODE_R+6};{_NODE_R+13}" '
                f'dur="2s" repeatCount="indefinite"/>'
                f'<animate attributeName="opacity" values="0.4;0" '
                f'dur="2s" repeatCount="indefinite"/>'
                f'</animate></circle>')

        parts.append(
            f'<circle class="topo-node" data-node="{name}" '
            f'cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

        if is_row:
            role_letter = nd['meta'].get('row_role', '?')[0].upper()
            row_id      = nd['meta'].get('row_id', '')
            parts.append(
                f'<text x="{cx:.1f}" y="{cy + 4:.1f}" '
                f'text-anchor="middle" fill="#0969da" '
                f'font-family="Courier New" font-size="8" font-weight="700" '
                f'style="pointer-events:none">{role_letter}{row_id}</text>')

        parts.append(
            f'<text x="{cx:.1f}" y="{cy + _NODE_R + 12:.1f}" '
            f'text-anchor="middle" fill="{label_col}" '
            f'font-family="Courier New" font-size="9" '
            f'style="pointer-events:none">{name}</text>')

    return (f'<svg id="topo-map" width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg">{"".join(parts)}</svg>')


# ── main node ─────────────────────────────────────────────────────────────────

class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')

        self.cmd_vel_publisher       = self.create_publisher(Twist, 'cmd_vel', 1)
        self.esp_enable_publisher    = self.create_publisher(Empty, 'esp/enable',    1)
        self.esp_disable_publisher   = self.create_publisher(Empty, 'esp/disable',   1)
        self.esp_reset_publisher     = self.create_publisher(Empty, 'esp/reset',     1)
        self.esp_restart_publisher   = self.create_publisher(Empty, 'esp/restart',   1)
        self.esp_configure_publisher = self.create_publisher(Empty, 'esp/configure', 1)
        self.subscription = self.create_subscription(
            GPSFix, 'gpsfix', self.store_gps, 1)
        self.battery_subscription = self.create_subscription(
            BatteryState, 'battery_state', self.store_battery, 1)
        self.bumper_front_top_subscription = self.create_subscription(
            Bool, 'bumper/front_top', self.update_bumper_front_top, SAFETY_QOS)
        self.bumper_front_bottom_subscription = self.create_subscription(
            Bool, 'bumper/front_bottom', self.update_bumper_front_bottom, SAFETY_QOS)
        self.bumper_back_subscription = self.create_subscription(
            Bool, 'bumper/back', self.update_bumper_back, SAFETY_QOS)
        self.estop_publisher = self.create_publisher(Bool, 'estop/soft', SAFETY_QOS)
        self.estop_front_subscription = self.create_subscription(
            Bool, 'estop/front', self.update_estop_front, SAFETY_QOS)
        self.estop_back_subscription = self.create_subscription(
            Bool, 'estop/back', self.update_estop_back, SAFETY_QOS)

        self._topo_doc:  dict            = {}
        self.topo_nodes: dict[str, dict] = _demo_nodes()
        self.topo_demo:  bool            = True

        self.create_subscription(
            String, '/topological_map_2', self._on_topo_map, TMAP_QOS)
        self._topo_map_pub = self.create_publisher(
            String, '/topological_map_2', TMAP_QOS)

        if _TOPO_SRV_OK:
            self._write_map_cli = self.create_client(
                WriteTopologicalMap,
                '/topological_map_manager2/write_topological_map')
            self._switch_map_cli = self.create_client(
                WriteTopologicalMap,
                '/topological_map_manager2/switch_topological_map')

        self.latest_odom: Optional[Odometry] = None
        self.create_subscription(
            Odometry, '/odometry/global',
            lambda m: setattr(self, 'latest_odom', m), 10)

        self.bumper_front_top_active    = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active         = False
        self.soft_estop_active          = False
        self.estop_front_active         = False
        self.estop_back_active          = False
        self.linear_velocity            = 0.0
        self.angular_velocity           = 0.0
        self.latest_gps: Optional[GPSFix]           = None
        self.latest_battery: Optional[BatteryState] = None

        self.topo_selected:   Optional[str] = None
        self.topo_current:    str           = '—'
        self.topo_nav_status: str           = 'Idle'
        self.topo_navigating: bool          = False
        self._nav_goal_handle              = None
        self.drop_status:     str           = ''

        self._track_timer:    Optional[object] = None
        self._track_counter:  int              = 0
        self.track_status:    str              = ''

        self.create_subscription(
            String, '/current_node',
            lambda m: setattr(self, 'topo_current', m.data), 10)

        if _ACTION_OK:
            self._nav_ac = ActionClient(self, GotoNode, 'topological_navigation')

        @ui.page('/')
        def page():
            self.content()

    # ── live map callback ─────────────────────────────────────────────────────

    def _on_topo_map(self, msg: String) -> None:
        try:
            nodes, doc      = _parse_topo_json(msg.data)
            self.topo_nodes = nodes
            self._topo_doc  = doc
            self.topo_demo  = False
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /topological_map_2: {e}')

    # ── topo nav actions ──────────────────────────────────────────────────────

    def send_nav_goal(self, target: str) -> None:
        if not _ACTION_OK:
            self.topo_nav_status = 'action server unavailable'
            return
        if not self._nav_ac.wait_for_server(timeout_sec=0.0):
            self.topo_nav_status = 'action server not ready'
            return
        goal = GotoNode.Goal()
        goal.target = target
        self.topo_nav_status = f'→ {target}'
        self.topo_navigating = True
        future = self._nav_ac.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._nav_accepted)

    def _nav_accepted(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.topo_nav_status = 'goal rejected'
            self.topo_navigating = False
            return
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
        # Sanitise: upper, spaces→underscore, strip invalid chars
        name = re.sub(r'[^A-Z0-9_]', '', name.strip().upper().replace(' ', '_'))
        if not name:
            self.drop_status = 'ERROR: node name required (letters/digits/underscores)'
            return
        if not _NAME_RE.match(name):
            self.drop_status = f'ERROR: invalid name "{name}" — use A-Z 0-9 _ only'
            return
        if name in self.topo_nodes:
            self.drop_status = f'ERROR: {name} already exists'
            return
        if self.latest_odom is None:
            self.drop_status = 'ERROR: no odometry — waiting for /odometry/global'
            return
        if not self._topo_doc:
            self.drop_status = 'ERROR: map not loaded'
            return

        x = round(self.latest_odom.pose.pose.position.x, 3)
        y = round(self.latest_odom.pose.pose.position.y, 3)

        connect_to = (self.topo_current
                      if self.topo_current not in ('—', 'none', 'None', '', None)
                      else None)
        map_name  = self._topo_doc.get('name', 'mixed_test_map')
        nav_frame = self._topo_doc.get('transformation', {}).get(
                        'topo_frame_id', 'map')

        is_row = row_id is not None
        if is_row:
            edge_action, xy_tol, yaw_tol, vert_r = _ROW_ACTION, 0.1, 0.05, 0.5
        else:
            edge_action, xy_tol, yaw_tol, vert_r = _NAV_ACTION, 0.3, 0.1, 1.0

        gps = self.latest_gps
        gps_meta: dict = {}
        if gps is not None and gps.status.status >= 0:
            gps_meta = {
                'gps_lat':      round(gps.latitude,  7),
                'gps_lon':      round(gps.longitude, 7),
                'gps_fix_type': int(gps.status.status),
                'gps_hdop':     round(float(gps.hdop), 2) if gps.hdop else None,
            }

        row_meta: dict = {}
        if is_row:
            row_meta = {'row_id': row_id, 'row_role': row_role or 'entry'}

        timestamp = datetime.now(timezone.utc).strftime('%d-%m-%Y_%H-%M-%S')

        node_meta_disk = {'map': map_name, 'node': name, 'pointset': map_name}
        node_meta_ui   = {**node_meta_disk, 'dropped_by': 'webui',
                          'timestamp': timestamp, **gps_meta, **row_meta}
        node_properties_disk = {
            'xy_goal_tolerance': xy_tol, 'yaw_goal_tolerance': yaw_tol,
            'dropped_by': 'webui', 'timestamp': timestamp, **gps_meta, **row_meta,
        }

        def _make_node_dict(meta: dict, props: dict) -> dict:
            return {
                'meta': meta,
                'node': {
                    'edges': ([{'action': edge_action,
                                'edge_id': f'{name}_{connect_to}',
                                'node': connect_to}] if connect_to else []),
                    'name':      name,
                    'nav_frame': nav_frame,
                    'pose': {
                        'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                        'position':    {'x': x,   'y': y,   'z': 0.0},
                    },
                    'properties': props,
                    'verts': [
                        {'x': -vert_r, 'y': -vert_r}, {'x': vert_r, 'y': -vert_r},
                        {'x':  vert_r, 'y':  vert_r}, {'x': -vert_r, 'y':  vert_r},
                    ],
                },
            }

        new_entry  = _make_node_dict(node_meta_ui,
                                     {'xy_goal_tolerance': xy_tol,
                                      'yaw_goal_tolerance': yaw_tol})
        new_nodes  = dict(self.topo_nodes)
        new_nodes[name] = {'x': x, 'y': y,
                           'edges': [connect_to] if connect_to else [],
                           'meta': node_meta_ui}

        if connect_to:
            if connect_to in new_nodes:
                rev = list(new_nodes[connect_to]['edges'])
                if name not in rev:
                    rev.append(name)
                new_nodes[connect_to] = {**new_nodes[connect_to], 'edges': rev}
            for entry in self._topo_doc.get('nodes', []):
                n = entry.get('node', {})
                if n.get('name') == connect_to:
                    n.setdefault('edges', []).append({
                        'action': edge_action,
                        'edge_id': f'{connect_to}_{name}',
                        'node': name,
                    })
                    break

        self._topo_doc.setdefault('nodes', []).append(new_entry)
        self.topo_nodes = new_nodes

        conn_str = f' → {connect_to}' if connect_to else ''
        gps_str  = (f' [{gps_meta["gps_lat"]:.5f},{gps_meta["gps_lon"]:.5f}]'
                    if gps_meta else '')
        row_str  = (f' row={row_id}/{row_role}' if is_row else '')
        self.drop_status = f'{name}{conn_str} at ({x}, {y}){row_str}{gps_str} — writing…'

        def _publish_and_persist():
            try:
                import copy
                import yaml as _yaml

                map_file      = f'/workspace/maps/{map_name}'
                installed_src = (
                    '/workspace/install/topological_navigation/share/'
                    'topological_navigation/config/mixed_actions_map.yaml')

                if os.path.exists(map_file):
                    with open(map_file) as f:
                        file_doc = _yaml.safe_load(f)
                elif os.path.exists(installed_src):
                    with open(installed_src) as f:
                        file_doc = _yaml.safe_load(f)
                    self.get_logger().info(
                        'mixed_test_map not found — seeding from installed source')
                else:
                    file_doc = copy.deepcopy(self._topo_doc)
                    self.get_logger().warn('No YAML source — using JSON round-trip fallback')

                disk_entry = _make_node_dict(node_meta_disk, node_properties_disk)
                file_doc.setdefault('nodes', []).append(disk_entry)

                if connect_to:
                    for entry in file_doc.get('nodes', []):
                        n = entry.get('node', {})
                        if n.get('name') == connect_to:
                            n.setdefault('edges', []).append({
                                'action': edge_action,
                                'edge_id': f'{connect_to}_{name}',
                                'node': name,
                            })
                            break

                with open(map_file, 'w') as f:
                    _yaml.dump(file_doc, f, default_flow_style=False,
                               allow_unicode=True, sort_keys=False)

                self.drop_status = (f'{name}{conn_str} at ({x}, {y})'
                                    f'{row_str}{gps_str} — reloading…')

                def _call(client, req, timeout=5.0):
                    ev = threading.Event()
                    res = [None]
                    def _cb(f): res[0] = f.result(); ev.set()
                    client.call_async(req).add_done_callback(_cb)
                    ev.wait(timeout=timeout)
                    return res[0]

                if _TOPO_SRV_OK:
                    sw          = WriteTopologicalMap.Request()
                    sw.filename = f'/workspace/maps/{map_name}'
                    sw.no_alias = True
                    sr = _call(self._switch_map_cli, sw)
                    if sr and sr.success:
                        self.drop_status = (f'{name}{conn_str} at ({x}, {y})'
                                            f'{row_str}{gps_str} — live')
                    else:
                        msg      = String()
                        msg.data = json.dumps(self._topo_doc, ensure_ascii=False)
                        self._topo_map_pub.publish(msg)
                        err = sr.message if sr else 'timeout'
                        self.drop_status = (f'{name}{conn_str} saved '
                                            f'(switch failed: {err})')
                        self.get_logger().warn(
                            f'switch_topological_map failed ({err}); published JSON directly')
                else:
                    msg      = String()
                    msg.data = json.dumps(self._topo_doc, ensure_ascii=False)
                    self._topo_map_pub.publish(msg)
                    self.drop_status = (f'{name}{conn_str} at ({x}, {y})'
                                        f'{row_str}{gps_str} — live (no srv)')

                self.get_logger().info(
                    f'Node dropped: {name} at ({x:.3f}, {y:.3f})'
                    f'{conn_str}{row_str}{gps_str}')
            except Exception as e:
                self.drop_status = f'ERROR: {e}'
                self.get_logger().error(f'drop_topo_node failed: {e}')

        threading.Thread(target=_publish_and_persist, daemon=True).start()

    # ── track mode ───────────────────────────────────────────────────────────

    def start_track(self, prefix: str, interval: float,
                    row_id: Optional[int], row_role: Optional[str]) -> None:
        prefix = re.sub(r'[^A-Z0-9_]', '', prefix.strip().upper().replace(' ', '_'))
        if not prefix:
            self.track_status = 'ERROR: prefix required'
            return
        if self._track_timer is not None:
            self.track_status = 'ERROR: already running'
            return

        existing = [
            n for n in self.topo_nodes
            if n.startswith(prefix + '_') and n[len(prefix) + 1:].isdigit()
        ]
        self._track_counter = (
            max(int(n[len(prefix) + 1:]) for n in existing) if existing else 0)

        def _drop() -> None:
            self._track_counter += 1
            self.drop_topo_node(f'{prefix}_{self._track_counter}', row_id, row_role)
            self.track_status = (f'recording  {prefix}_{self._track_counter}'
                                 f'  (#{self._track_counter})')

        _drop()
        self._track_timer = self.create_timer(interval, _drop)

    def stop_track(self) -> None:
        if self._track_timer is not None:
            self._track_timer.cancel()
            self._track_timer = None
        self.track_status = f'stopped at #{self._track_counter}'
        self._track_counter = 0

    # ── UI ────────────────────────────────────────────────────────────────────

    def content(self) -> None:
        ui.add_head_html(_GLOBAL_CSS)
        with ui.tabs().classes('w-full') as tabs:
            tab_navigate = ui.tab('Navigate', icon='route')
            tab_control  = ui.tab('Control',  icon='settings')

        with ui.tab_panels(tabs, value=tab_navigate).classes('w-full'):
            with ui.tab_panel(tab_navigate):
                self._nav_content()
            with ui.tab_panel(tab_control):
                self._control_content()

    # ── Navigate tab ─────────────────────────────────────────────────────────

    def _nav_content(self) -> None:
        # Top status row
        with ui.row().classes('items-center gap-3 mb-3 w-full'):
            ui.label('Sowbot').classes('text-lg font-semibold')
            map_pill   = ui.html('')
            action_pill = ui.html('')

        # Main layout: map + sidebar
        with ui.row().classes('w-full gap-3 items-start'):

            # Left column: map + controls stacked
            with ui.column().classes('flex-1 gap-3').style('min-width:0'):

                # Topo map
                with ui.card().style('padding:8px'):
                    map_html = ui.html(_build_svg(self.topo_nodes, None, None))

                # Joystick + E-Stop
                with ui.card().classes('w-full'):
                    with ui.row().classes('items-center gap-6 justify-center'):
                        ui.joystick(color='#1a7f37', size=48,
                                    on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                                    on_end=lambda _: self.send_speed(0.0, 0.0))
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
                            'color=primary outline no-caps').classes('w-28 h-12')
                        pose_lbl = ui.label('—').classes('text-xs font-mono').style(
                            'color:#57606a')

                # Drop Node
                with ui.card().classes('w-full'):
                    with ui.row().classes('items-baseline gap-2 mb-2'):
                        ui.label('Drop Node').classes('font-semibold')
                        ui.label('pins at current pose').classes('text-xs').style(
                            'color:#8c959f')

                    with ui.row().classes('items-center gap-3 w-full'):
                        name_input = ui.input(
                            placeholder='e.g. ROW_D_IN', label='Name',
                        ).classes('flex-1')

                    with ui.row().classes('items-center gap-3 w-full mt-1'):
                        row_id_input = ui.number(
                            label='Row ID', placeholder='blank = standard',
                            min=1, step=1, precision=0,
                        ).classes('w-32')
                        row_role_toggle = ui.toggle(
                            {'entry': 'Entry', 'exit': 'Exit'}, value='entry',
                        ).props('dense')
                        row_hint = ui.label('').classes('text-xs font-mono').style(
                            'color:#8c959f')

                    with ui.row().classes('items-center gap-3 mt-2'):
                        cur_drop_lbl = ui.label('').classes('text-xs font-mono').style(
                            'color:#8c959f')
                        ui.button('Drop Node', on_click=lambda: self.drop_topo_node(
                            name_input.value,
                            int(row_id_input.value) if row_id_input.value else None,
                            row_role_toggle.value,
                        )).classes('ml-auto').props('color=positive no-caps')

                    drop_status_lbl = ui.label('').classes('text-xs font-mono mt-1')

                # Track Mode
                with ui.card().classes('w-full'):
                    with ui.row().classes('items-baseline gap-2 mb-2'):
                        ui.label('Track Mode').classes('font-semibold')
                        ui.label('auto-drop every N seconds').classes('text-xs').style(
                            'color:#8c959f')

                    with ui.row().classes('items-center gap-3 w-full'):
                        track_prefix = ui.input(
                            placeholder='e.g. TOP_FIELD', label='Prefix',
                        ).classes('flex-1')
                        track_interval = ui.number(
                            label='s', value=5, min=2, max=30, step=1, precision=0,
                        ).classes('w-20')

                    with ui.row().classes('items-center gap-3 w-full mt-1'):
                        track_row_id = ui.number(
                            label='Row ID', placeholder='blank = standard',
                            min=1, step=1, precision=0,
                        ).classes('w-32')
                        track_row_role = ui.toggle(
                            {'entry': 'Entry', 'exit': 'Exit'}, value='entry',
                        ).props('dense')
                        track_row_hint = ui.label('').classes('text-xs font-mono').style(
                            'color:#8c959f')

                    with ui.row().classes('items-center gap-2 mt-2'):
                        track_start_btn = ui.button('Start', on_click=lambda: self.start_track(
                            track_prefix.value,
                            float(track_interval.value or 5),
                            int(track_row_id.value) if track_row_id.value else None,
                            track_row_role.value,
                        )).props('color=positive no-caps')
                        track_stop_btn = ui.button('Stop', on_click=self.stop_track).props(
                            'color=negative no-caps')
                        track_status_lbl = ui.label('').classes('text-xs font-mono ml-1').style(
                            'color:#57606a')

            # Right sidebar: nav controls + node list
            with ui.card().style('width:200px;padding:12px;flex-shrink:0;'
                                  'display:flex;flex-direction:column;gap:8px'):

                ui.html('<div class="sec-label">Current node</div>')
                cur_lbl = ui.label('—').classes('text-sm font-mono font-bold')

                ui.html('<div class="sec-label mt-2">Destination</div>')
                sel_lbl = ui.label('—').classes('text-sm font-mono').style('color:#8c959f')

                ui.html('<div class="sec-label mt-2">Status</div>')
                stat_lbl = ui.label('idle').classes('text-xs font-mono').style('color:#57606a')

                ui.separator()

                go_btn   = ui.button('Go', color='positive').classes('w-full').props('no-caps')
                stop_btn = ui.button('Cancel', color='negative').classes('w-full').props(
                    'no-caps flat')

                ui.html('<div class="sec-label mt-2">Nodes</div>')
                with ui.scroll_area().style('height:200px'):
                    node_col = ui.column().style('gap:1px;width:100%')

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
            # Pose / connect label
            odom = self.latest_odom
            gps  = self.latest_gps
            if odom is not None:
                px = odom.pose.pose.position.x
                py = odom.pose.pose.position.y
                gps_str = (f'  {gps.latitude:.5f},{gps.longitude:.5f}'
                           if gps and gps.status.status >= 0 else '')
                pose_lbl.set_text(f'({px:.2f}, {py:.2f}){gps_str}')
            else:
                pose_lbl.set_text('no odometry')

            cur = self.topo_current
            if cur and cur != '—':
                cur_drop_lbl.set_text(f'connects to {cur}')
                cur_drop_lbl.style('color:#1a7f37')
            else:
                cur_drop_lbl.set_text('no current node — edge skipped')
                cur_drop_lbl.style('color:#8c959f')

            if row_id_input.value:
                row_hint.set_text(f'action: {_ROW_ACTION}  tol: 0.1 m')
                row_hint.style('color:#0969da')
            else:
                row_hint.set_text(f'action: {_NAV_ACTION}')
                row_hint.style('color:#8c959f')

            drop_status_lbl.set_text(self.drop_status)
            drop_status_lbl.style(
                'color:#cf222e' if self.drop_status.startswith('ERROR')
                else 'color:#1a7f37')

            # Track
            running = self._track_timer is not None
            track_start_btn.set_enabled(not running)
            track_stop_btn.set_enabled(running)
            if track_row_id.value:
                track_row_hint.set_text(f'action: {_ROW_ACTION}')
                track_row_hint.style('color:#0969da')
            else:
                track_row_hint.set_text(f'action: {_NAV_ACTION}')
                track_row_hint.style('color:#8c959f')
            track_status_lbl.set_text(self.track_status)
            track_status_lbl.style(
                'color:#cf222e' if self.track_status.startswith('ERROR') else
                'color:#1a7f37' if running else 'color:#57606a')

            snap = {
                'sel':   self.topo_selected,
                'cur':   self.topo_current,
                'stat':  self.topo_nav_status,
                'nav':   self.topo_navigating,
                'nodes': id(self.topo_nodes),
                'demo':  self.topo_demo,
            }
            changed = {k for k, v in snap.items() if _prev.get(k) != v}
            if not changed:
                return
            _prev.update(snap)

            if 'demo' in changed or not _prev:
                map_pill.set_content(
                    f'<span class="pill {"pill-ok" if not self.topo_demo else "pill-warn"}">'
                    f'{"live" if not self.topo_demo else "demo"}</span>')
                action_pill.set_content(
                    f'<span class="pill {"pill-ok" if _ACTION_OK else "pill-off"}">'
                    f'{"action ok" if _ACTION_OK else "no action"}</span>')

            if changed & {'sel', 'cur', 'nodes'}:
                map_html.set_content(
                    _build_svg(self.topo_nodes, self.topo_selected, self.topo_current))
                inject_click_js()
                node_col.clear()
                with node_col:
                    for nname in sorted(self.topo_nodes):
                        nd       = self.topo_nodes[nname]
                        is_sel   = (nname == self.topo_selected)
                        is_row   = nd.get('meta', {}).get('row_id') is not None
                        row_id_v = nd.get('meta', {}).get('row_id', '')
                        row_role_v = nd.get('meta', {}).get('row_role', '')
                        gps_lat  = nd.get('meta', {}).get('gps_lat', '')
                        gps_lon  = nd.get('meta', {}).get('gps_lon', '')
                        title    = (f'Row {row_id_v} {row_role_v} | {gps_lat} {gps_lon}'.strip()
                                    if is_row else f'{gps_lat} {gps_lon}'.strip())
                        cls = ('node-item'
                               + (' sel' if is_sel else '')
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
                'color:#1a7f37' if self.topo_nav_status == 'arrived' else
                'color:#57606a')

            can_go = (bool(self.topo_selected) and not self.topo_navigating
                      and not self.soft_estop_active)
            go_btn.set_enabled(can_go)
            stop_btn.set_enabled(self.topo_navigating)

        ui.timer(0.2, refresh_nav)
        inject_click_js()

    # ── Control tab ───────────────────────────────────────────────────────────

    def _control_content(self) -> None:
        with ui.row().classes('items-stretch w-[48rem] gap-3'):
            with ui.card().classes('flex-1'):
                ui.label('Telemetry').classes('font-semibold mb-2')
                ui.html('<div class="sec-label">Linear velocity</div>')
                ui.slider(min=-1, max=1, step=0.05, value=0).props(
                    'readonly selection-color=transparent color=green'
                ).bind_value(self, 'linear_velocity')
                ui.html('<div class="sec-label mt-2">Angular velocity</div>')
                ui.slider(min=-1, max=1, step=0.05, value=0).props(
                    'readonly selection-color=transparent color=green'
                ).bind_value(self, 'angular_velocity')
                ui.html('<div class="sec-label mt-3">Battery</div>')
                ui.label().classes('text-sm').bind_text_from(
                    self, 'latest_battery',
                    lambda msg: (f'{msg.percentage * 100:.1f}%  {msg.voltage:.1f} V'
                                 if msg is not None else '—'))

            with ui.card().classes('flex-1'):
                ui.label('Safety').classes('font-semibold mb-2')
                ui.html('<div class="sec-label">Bumpers</div>')
                for attr, label in [
                    ('bumper_front_top_active',    'Front top'),
                    ('bumper_front_bottom_active', 'Front bottom'),
                    ('bumper_back_active',          'Rear'),
                ]:
                    with ui.row().classes('items-center gap-0'):
                        dot = ui.html('<span class="dot-off"></span>')
                        ui.label(label).classes('text-sm')
                    def _mk(d=dot, a=attr):
                        def _u():
                            d.set_content(
                                f'<span class="dot-{"warn" if getattr(self, a) else "ok"}"></span>')
                        return _u
                    ui.timer(0.2, _mk())

                ui.html('<div class="sec-label mt-3">E-stops</div>')
                for attr, label in [
                    ('estop_front_active', 'Front'),
                    ('estop_back_active',  'Rear'),
                ]:
                    with ui.row().classes('items-center gap-0'):
                        dot = ui.html('<span class="dot-off"></span>')
                        ui.label(label).classes('text-sm')
                    def _mk2(d=dot, a=attr):
                        def _u():
                            d.set_content(
                                f'<span class="dot-{"warn" if getattr(self, a) else "off"}"></span>')
                        return _u
                    ui.timer(0.2, _mk2())

        with ui.card().classes('w-[48rem] mt-3'):
            ui.label('ESP').classes('font-semibold mb-2')
            with ui.row().classes('gap-2 flex-wrap'):
                ui.button('Enable',    on_click=lambda: self.esp_enable_publisher.publish(Empty())).props('color=positive outline no-caps').classes('px-4')
                ui.button('Disable',   on_click=lambda: self.esp_disable_publisher.publish(Empty())).props('color=negative outline no-caps').classes('px-4')
                ui.button('Reset',     on_click=lambda: self.esp_reset_publisher.publish(Empty())).props('color=warning outline no-caps').classes('px-4')
                ui.button('Restart',   on_click=lambda: self.esp_restart_publisher.publish(Empty())).props('color=primary outline no-caps').classes('px-4')
                ui.button('Configure', on_click=lambda: self.esp_configure_publisher.publish(Empty())).props('outline no-caps').classes('px-4')

        with ui.card().classes('w-[48rem] mt-3'):
            ui.label('GPS').classes('font-semibold mb-2')
            leaflet = ui.leaflet(center=(51.98278, 7.43440), zoom=16).classes('w-full h-80')
            marker  = leaflet.marker(latlng=leaflet.center)
            def update_gps_ui() -> None:
                if self.latest_gps is not None:
                    leaflet.set_center((self.latest_gps.latitude, self.latest_gps.longitude))
                    marker.move(self.latest_gps.latitude, self.latest_gps.longitude)
            ui.timer(2.0, update_gps_ui)

    # ── shared methods ────────────────────────────────────────────────────────

    def toggle_estop(self) -> None:
        self.soft_estop_active = not self.soft_estop_active
        msg = Bool()
        msg.data = self.soft_estop_active
        self.estop_publisher.publish(msg)

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear_velocity = x
        self.angular_velocity = y
        self.cmd_vel_publisher.publish(msg)

    def store_gps(self, msg: GPSFix) -> None:        self.latest_gps = msg
    def store_battery(self, msg: BatteryState) -> None: self.latest_battery = msg
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
