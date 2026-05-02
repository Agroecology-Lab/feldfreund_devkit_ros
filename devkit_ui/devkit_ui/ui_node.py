# pylint: disable=duplicate-code
"""
ui_node.py — Sowbot web cockpit on :80

Tabs
  Control   — joystick, e-stop, ESP buttons, GPS map, drop topo node
  Navigate  — topological map, node picker, GO / CANCEL

Map source: /topological_map_2 topic (std_msgs/String JSON) from
topological_map_manager_2. No TMAP2_FILE env var needed.

Node drop workflow:
  1. Validate name + pose
  2. Add node to _topo_doc in memory
  3. Write updated YAML to /workspace/mixed_test_map
  4. Call switch_topological_map so manager reloads the file,
     republishes /topological_map_2, triggers RViz redraw + KD-tree rebuild.
  5. On switch failure, fall back to publishing JSON directly to the topic.
"""

import json
import os
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

# QoS matching map_manager2's /topological_map_2 publisher (latched)
TMAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


# ── live map parser ───────────────────────────────────────────────────────────

def _parse_topo_json(json_str: str) -> tuple[dict[str, dict], dict]:
    """
    Parse JSON from /topological_map_2.
    Returns (nodes, full_doc):
      nodes    = {name: {x, y, edges: [name, ...], meta: {...}}}
      full_doc = full parsed document for round-trip publish/write
    """
    doc = json.loads(json_str)
    nodes: dict[str, dict] = {}
    for entry in doc.get('nodes', []):
        n     = entry['node']
        name  = n['name']
        pos   = n['pose']['position']
        edges = [e['node'] for e in n.get('edges', []) if 'node' in e]
        meta  = entry.get('meta', {})
        nodes[name] = {
            'x':     float(pos['x']),
            'y':     float(pos['y']),
            'edges': edges,
            'meta':  meta,
        }
    return nodes, doc


def _demo_nodes() -> dict[str, dict]:
    """Fallback 6-node line matching mixed_test_map layout — shown before
    /topological_map_2 is received."""
    return {
        'N1': {'x':  0.0, 'y': 0.0, 'edges': ['N2'],       'meta': {}},
        'N2': {'x':  3.0, 'y': 0.0, 'edges': ['N1', 'N3'], 'meta': {}},
        'N3': {'x':  6.0, 'y': 0.0, 'edges': ['N2', 'N4'], 'meta': {}},
        'N4': {'x':  9.0, 'y': 0.0, 'edges': ['N3', 'N5'], 'meta': {}},
        'N5': {'x': 12.0, 'y': 0.0, 'edges': ['N4', 'N6'], 'meta': {}},
        'N6': {'x': 15.0, 'y': 0.0, 'edges': ['N5'],       'meta': {}},
    }


# ── SVG map renderer ──────────────────────────────────────────────────────────

_SVG_W, _SVG_H = 720, 380
_MARGIN, _NODE_R = 48, 20


def _build_svg(nodes: dict, selected: Optional[str], current: Optional[str]) -> str:
    if not nodes:
        return (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
                f'style="background:#0a160a;border-radius:6px">'
                f'<text x="{_SVG_W//2}" y="{_SVG_H//2}" text-anchor="middle" '
                f'fill="#4ade80" font-family="monospace" font-size="16">'
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
        f'<rect width="{_SVG_W}" height="{_SVG_H}" fill="#0a160a" rx="6"/>']

    drawn: set = set()
    for name, nd in nodes.items():
        for tgt in nd['edges']:
            key = tuple(sorted([name, tgt]))
            if key in drawn or tgt not in nodes:
                continue
            drawn.add(key)
            parts.append(
                f'<line x1="{tx(nd["x"]):.1f}" y1="{ty(nd["y"]):.1f}" '
                f'x2="{tx(nodes[tgt]["x"]):.1f}" y2="{ty(nodes[tgt]["y"]):.1f}" '
                f'stroke="#1e3320" stroke-width="3" stroke-linecap="round"/>')

    for name, nd in nodes.items():
        cx, cy   = tx(nd['x']), ty(nd['y'])
        is_cur   = (name == current)
        is_sel   = (name == selected)
        is_drop  = bool(nd.get('meta', {}).get('dropped_by'))
        fill     = '#16a34a' if is_cur else ('#f59e0b' if is_sel else
                   ('#1a2e1a' if is_drop else '#162616'))
        stroke   = '#86efac' if is_cur else ('#fde68a' if is_sel else
                   ('#f59e0b' if is_drop else '#22c55e'))
        sw       = 4 if (is_cur or is_sel) else 2
        lc       = '#ffffff' if is_cur else ('#fde68a' if is_sel else '#86efac')

        if is_cur:
            parts.append(
                f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R + 8}" '
                f'fill="none" stroke="#86efac" stroke-width="2" opacity="0.35">'
                f'<animate attributeName="r" values="{_NODE_R+8};{_NODE_R+16}" '
                f'dur="1.6s" repeatCount="indefinite"/>'
                f'<animate attributeName="opacity" values="0.35;0" '
                f'dur="1.6s" repeatCount="indefinite"/>'
                f'</animate></circle>')

        parts.append(
            f'<circle class="topo-node" data-node="{name}" '
            f'cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
            f'style="cursor:pointer"/>')
        parts.append(
            f'<text x="{cx:.1f}" y="{cy + _NODE_R + 14:.1f}" '
            f'text-anchor="middle" fill="{lc}" '
            f'font-family="monospace" font-size="10" font-weight="bold" '
            f'style="pointer-events:none">{name.replace("_", " ")}</text>')

    return (f'<svg id="topo-map" width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg">{"".join(parts)}</svg>')


# ── main node ─────────────────────────────────────────────────────────────────

class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')

        # ── publishers / subscriptions ────────────────────────────────────────
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

        # ── live topo map ─────────────────────────────────────────────────────
        self._topo_doc:  dict            = {}
        self.topo_nodes: dict[str, dict] = _demo_nodes()
        self.topo_demo:  bool            = True

        # Subscribe to live map (TRANSIENT_LOCAL so we get it on connect)
        self.create_subscription(
            String, '/topological_map_2', self._on_topo_map, TMAP_QOS)

        # Publisher to push map updates — same QoS as map_manager2
        self._topo_map_pub = self.create_publisher(
            String, '/topological_map_2', TMAP_QOS)

        # ── map manager service clients ───────────────────────────────────────
        if _TOPO_SRV_OK:
            self._write_map_cli = self.create_client(
                WriteTopologicalMap,
                '/topological_map_manager2/write_topological_map')
            self._switch_map_cli = self.create_client(
                WriteTopologicalMap,
                '/topological_map_manager2/switch_topological_map')

        # ── odometry (fake_nav2_server in sim; fusioncore /fusion/odom in field)
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

        # ── topo nav state ────────────────────────────────────────────────────
        self.topo_selected:   Optional[str] = None
        self.topo_current:    str           = '—'
        self.topo_nav_status: str           = 'Idle'
        self.topo_navigating: bool          = False
        self._nav_goal_handle              = None
        self.drop_status:     str           = ''

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
            self.topo_nodes = nodes   # new dict → id() change triggers redraw
            self._topo_doc  = doc
            self.topo_demo  = False
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /topological_map_2: {e}')

    # ── topo nav actions ──────────────────────────────────────────────────────

    def send_nav_goal(self, target: str) -> None:
        if not _ACTION_OK:
            self.topo_nav_status = 'ERROR: topological_navigation_msgs not installed'
            return
        if not self._nav_ac.wait_for_server(timeout_sec=0.0):
            self.topo_nav_status = 'ERROR: action server not available'
            return
        goal = GotoNode.Goal()
        goal.target = target
        self.topo_nav_status = f'Navigating → {target}'
        self.topo_navigating = True
        future = self._nav_ac.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._nav_accepted)

    def _nav_accepted(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.topo_nav_status = 'Goal rejected'
            self.topo_navigating = False
            return
        self._nav_goal_handle = gh
        gh.get_result_async().add_done_callback(self._nav_result)

    def _nav_feedback(self, feedback_msg) -> None:
        fb  = feedback_msg.feedback
        loc = getattr(fb, 'current_node', None) or getattr(fb, 'status', '…')
        self.topo_nav_status = f'En route · {loc}'

    def _nav_result(self, future) -> None:
        success = getattr(future.result().result, 'success', True)
        self.topo_nav_status = 'Arrived' if success else 'Navigation failed'
        self.topo_navigating = False
        self._nav_goal_handle = None

    def cancel_nav_goal(self) -> None:
        if self._nav_goal_handle:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self.topo_nav_status = 'Cancelled'
        self.topo_navigating = False

    # ── node dropping ─────────────────────────────────────────────────────────

    def drop_topo_node(self, name: str) -> None:
        """
        Pin a new topo node at the current robot pose.

        Write sequence:
          1. Load base YAML: /workspace/mixed_test_map  (exists after first drop)
                         OR: installed mixed_actions_map.yaml (fresh container)
                         OR: _topo_doc JSON round-trip (last-resort fallback)
          2. Append new node + patch reverse edge on connect_to
          3. yaml.dump → /workspace/mixed_test_map
          4. switch_topological_map('mixed_test_map')
             → manager reloads file → republishes /topological_map_2
             → visualiser redraws RViz markers
             → localisation rebuilds KD-tree
          5. On switch failure: publish JSON directly to /topological_map_2
             so the session still has the updated map, and log a warning.
        """
        name = name.strip().upper().replace(' ', '_')
        if not name:
            self.drop_status = 'ERROR: enter a node name'
            return
        if name in self.topo_nodes:
            self.drop_status = f'ERROR: {name} already exists'
            return
        if self.latest_odom is None:
            self.drop_status = 'ERROR: no pose — waiting for /odometry/global'
            return
        if not self._topo_doc:
            self.drop_status = 'ERROR: map not loaded yet'
            return

        x = round(self.latest_odom.pose.pose.position.x, 3)
        y = round(self.latest_odom.pose.pose.position.y, 3)

        connect_to = (self.topo_current
                      if self.topo_current not in ('—', 'none', 'None', '', None)
                      else None)
        map_name   = self._topo_doc.get('name', 'mixed_test_map')
        nav_frame  = self._topo_doc.get('transformation', {}).get(
                         'topo_frame_id', 'map')

        # ── GPS metadata ──────────────────────────────────────────────────────
        gps = self.latest_gps
        gps_meta: dict = {}
        if gps is not None and gps.status.status >= 0:
            gps_meta = {
                'gps_lat':      round(gps.latitude,  7),
                'gps_lon':      round(gps.longitude, 7),
                'gps_fix_type': int(gps.status.status),
                'gps_hdop':     round(float(gps.hdop), 2) if gps.hdop else None,
            }

        node_meta = {
            'map':        map_name,
            'node':       name,
            'pointset':   map_name,
            'dropped_by': 'webui',
            'timestamp':  datetime.now(timezone.utc).strftime('%d-%m-%Y_%H-%M-%S'),
            **gps_meta,
        }

        # ── build node entry matching mixed_test_map schema exactly ───────────
        new_entry = {
            'meta': node_meta,
            'node': {
                'edges': ([{
                    'action':  'navigate_to_pose',
                    'edge_id': f'{name}_{connect_to}',
                    'node':    connect_to,
                }] if connect_to else []),
                'name':      name,
                'nav_frame': nav_frame,
                'pose': {
                    'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'position':    {'x': x,   'y': y,   'z': 0.0},
                },
                'properties': {'xy_goal_tolerance': 0.3, 'yaw_goal_tolerance': 0.1},
                'verts': [
                    {'x': -1.0, 'y': -1.0}, {'x': 1.0, 'y': -1.0},
                    {'x':  1.0, 'y':  1.0}, {'x': -1.0, 'y':  1.0},
                ],
            },
        }

        # ── update in-memory UI map ───────────────────────────────────────────
        new_nodes = dict(self.topo_nodes)
        new_nodes[name] = {
            'x': x, 'y': y,
            'edges': [connect_to] if connect_to else [],
            'meta':  node_meta,
        }

        # patch reverse edge on connect_to in both UI nodes and full doc
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
                        'action':  'navigate_to_pose',
                        'edge_id': f'{connect_to}_{name}',
                        'node':    name,
                    })
                    break

        self._topo_doc.setdefault('nodes', []).append(new_entry)
        self.topo_nodes = new_nodes   # new dict → triggers nav tab SVG redraw

        conn_str = f' → {connect_to}' if connect_to else ''
        gps_str  = (f' GPS({gps_meta["gps_lat"]:.5f},{gps_meta["gps_lon"]:.5f})'
                    if gps_meta else '')
        self.drop_status = f'✓ {name}{conn_str} at ({x}, {y}){gps_str} — publishing…'

        def _publish_and_persist():
            try:
                import copy
                import yaml as _yaml

                # ── Step 1: load base document ────────────────────────────────
                # Priority order:
                #   a) /workspace/mixed_test_map          — exists after first drop
                #   b) installed mixed_actions_map.yaml   — fresh container start
                #   c) _topo_doc JSON round-trip          — last-resort fallback
                #
                # Using the on-disk YAML (a or b) preserves the BT action
                # definitions block that the map manager needs for a clean
                # switch_topological_map reload. A JSON round-trip (c) loses
                # any YAML-only structure and can cause the switch to fail.
                map_file      = f'/workspace/{map_name}'
                installed_src = (
                    '/workspace/install/topological_navigation/share/'
                    'topological_navigation/config/mixed_actions_map.yaml'
                )

                if os.path.exists(map_file):
                    with open(map_file) as f:
                        file_doc = _yaml.safe_load(f)
                elif os.path.exists(installed_src):
                    with open(installed_src) as f:
                        file_doc = _yaml.safe_load(f)
                    self.get_logger().info(
                        f'mixed_test_map not found — seeding from installed source')
                else:
                    # Neither file exists (shouldn't happen in normal deployment).
                    # Fall back to the JSON-parsed in-memory doc.
                    file_doc = copy.deepcopy(self._topo_doc)
                    self.get_logger().warn(
                        'No YAML source found — using JSON round-trip fallback')

                # ── Step 2: append node + patch reverse edge ──────────────────
                new_node_entry = {
                    'meta': node_meta,
                    'node': {
                        'edges': ([{
                            'action':  'navigate_to_pose',
                            'edge_id': f'{name}_{connect_to}',
                            'node':    connect_to,
                        }] if connect_to else []),
                        'name':      name,
                        'nav_frame': nav_frame,
                        'pose': {
                            'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                            'position':    {'x': x,   'y': y,   'z': 0.0},
                        },
                        'properties': {'xy_goal_tolerance': 0.3, 'yaw_goal_tolerance': 0.1},
                        'verts': [
                            {'x': -1.0, 'y': -1.0}, {'x': 1.0, 'y': -1.0},
                            {'x':  1.0, 'y':  1.0}, {'x': -1.0, 'y':  1.0},
                        ],
                    },
                }

                file_doc.setdefault('nodes', []).append(new_node_entry)
                if connect_to:
                    for entry in file_doc.get('nodes', []):
                        n = entry.get('node', {})
                        if n.get('name') == connect_to:
                            n.setdefault('edges', []).append({
                                'action':  'navigate_to_pose',
                                'edge_id': f'{connect_to}_{name}',
                                'node':    name,
                            })
                            break

                # ── Step 3: write YAML to disk ────────────────────────────────
                with open(map_file, 'w') as f:
                    _yaml.dump(
                        file_doc, f,
                        default_flow_style=False,
                        allow_unicode=True,
                        sort_keys=False,
                    )

                self.drop_status = (f'✓ {name}{conn_str} at ({x}, {y})'
                                    f'{gps_str} — written, reloading…')

                # ── Step 4: switch reloads updated file into map manager ───────
                # Manager republishes /topological_map_2 → visualiser redraws
                # RViz markers → localisation rebuilds KD-tree.
                # topological_map2_path="" so manager uses req.filename directly
                # as path from its working dir (/workspace).
                def _call(client, req, timeout=5.0):
                    ev = threading.Event()
                    res = [None]
                    def _cb(f): res[0] = f.result(); ev.set()
                    client.call_async(req).add_done_callback(_cb)
                    ev.wait(timeout=timeout)
                    return res[0]

                if _TOPO_SRV_OK:
                    sw          = WriteTopologicalMap.Request()
                    sw.filename = map_name   # 'mixed_test_map' → /workspace/mixed_test_map
                    sw.no_alias = True
                    sr = _call(self._switch_map_cli, sw)
                    if sr and sr.success:
                        self.drop_status = (f'✓ {name}{conn_str} at ({x}, {y})'
                                            f'{gps_str} — live in RViz + saved')
                    else:
                        # Switch failed — file is on disk for next restart, but
                        # publish directly so this session still sees the node.
                        msg      = String()
                        msg.data = json.dumps(self._topo_doc, ensure_ascii=False)
                        self._topo_map_pub.publish(msg)
                        err = sr.message if sr else 'timeout'
                        self.drop_status = (f'✓ {name}{conn_str} saved to disk '
                                            f'(switch failed: {err} '
                                            f'— published to topic as fallback)')
                        self.get_logger().warn(
                            f'switch_topological_map failed ({err}); '
                            f'published JSON directly — RViz may not update until restart')
                else:
                    msg      = String()
                    msg.data = json.dumps(self._topo_doc, ensure_ascii=False)
                    self._topo_map_pub.publish(msg)
                    self.drop_status = (f'✓ {name}{conn_str} at ({x}, {y})'
                                        f'{gps_str} — live (no write srv)')

                self.get_logger().info(
                    f'Node dropped: {name} at ({x:.3f}, {y:.3f}){conn_str}{gps_str}')
            except Exception as e:
                self.drop_status = f'ERROR: {e}'
                self.get_logger().error(f'drop_topo_node failed: {e}')

        threading.Thread(target=_publish_and_persist, daemon=True).start()

    # ── UI ────────────────────────────────────────────────────────────────────

    def content(self) -> None:
        with ui.tabs().classes('w-full') as tabs:
            tab_control  = ui.tab('Control',  icon='gamepad')
            tab_navigate = ui.tab('Navigate', icon='route')

        with ui.tab_panels(tabs, value=tab_control).classes('w-full'):
            with ui.tab_panel(tab_control):
                self._control_content()
            with ui.tab_panel(tab_navigate):
                self._nav_content()

    def _control_content(self) -> None:
        with ui.row().classes('items-stretch w-[48rem] gap-3'):
            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Control').classes('text-2xl')
                ui.joystick(color='blue', size=50,
                            on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                            on_end=lambda _: self.send_speed(0.0, 0.0))
                ui.label('Publish steering commands by dragging your mouse '
                         'around in the blue field').classes('mt-6')

                def update_button_appearance(e: ClickEventArguments) -> None:
                    print(f'update_button_appearance: {e}')
                    assert isinstance(e.sender, ui.button)
                    self.toggle_estop()
                    if self.soft_estop_active:
                        e.sender.props('color=red')
                        e.sender.text = 'STOPPED'
                    else:
                        e.sender.props('color=blue')
                        e.sender.text = 'EMERGENCY STOP'
                ui.button('EMERGENCY STOP', color='blue',
                          on_click=update_button_appearance).classes('w-40 min-h-[3rem]')

            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Data').classes('text-2xl')
                ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                slider_props = 'readonly selection-color=transparent'
                ui.slider(min=-1, max=1, step=0.05, value=0) \
                    .props(slider_props).bind_value(self, 'linear_velocity')
                ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                ui.slider(min=-1, max=1, step=0.05, value=0) \
                    .props(slider_props).bind_value(self, 'angular_velocity')
                ui.label('Battery').classes('text-xs mb-[-1.4em]')
                ui.label().bind_text_from(
                    self, 'latest_battery',
                    lambda msg: (f'{msg.percentage * 100:.1f}% ({msg.voltage:.1f}V)'
                                 if msg is not None else 'N/A'))

            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Safety').classes('text-2xl')
                ui.label('Bumpers').classes('text-xs mb-[-1.4em]')
                ui.label().classes('text-sm').bind_text_from(
                    self, 'bumper_front_top_active',
                    lambda a: 'Front Top: ' + ('ACTIVE' if a else 'inactive'))
                ui.label().classes('text-sm').bind_text_from(
                    self, 'bumper_front_bottom_active',
                    lambda a: 'Front Bottom: ' + ('ACTIVE' if a else 'inactive'))
                ui.label().classes('text-sm').bind_text_from(
                    self, 'bumper_back_active',
                    lambda a: 'Back: ' + ('ACTIVE' if a else 'inactive'))
                ui.label('E-Stops').classes('text-xs mb-[-1.4em] mt-4')
                ui.label().classes('text-sm').bind_text_from(
                    self, 'estop_front_active',
                    lambda a: 'Front: ' + ('ACTIVE' if a else 'inactive'))
                ui.label().classes('text-sm').bind_text_from(
                    self, 'estop_back_active',
                    lambda a: 'Back: ' + ('ACTIVE' if a else 'inactive'))

        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('ESP Control').classes('text-2xl')
            with ui.row().classes('gap-4'):
                ui.button('Enable',    color='green',
                          on_click=lambda: self.esp_enable_publisher.publish(Empty())).classes('w-24')
                ui.button('Disable',   color='red',
                          on_click=lambda: self.esp_disable_publisher.publish(Empty())).classes('w-24')
                ui.button('Reset',     color='orange',
                          on_click=lambda: self.esp_reset_publisher.publish(Empty())).classes('w-24')
                ui.button('Restart',   color='blue',
                          on_click=lambda: self.esp_restart_publisher.publish(Empty())).classes('w-24')
                ui.button('Configure', color='purple',
                          on_click=lambda: self.esp_configure_publisher.publish(Empty())).classes('w-24')

        # ── Drop Topo Node card ───────────────────────────────────────────────
        with ui.card().classes('w-[48rem] mt-3'):
            ui.label('Drop Topo Node').classes('text-2xl')
            ui.label(
                'Stop the robot, enter a name, press DROP. Node is pinned at '
                'the current pose, connected to the current topo node, '
                'published live to all nav stack subscribers, and saved to disk.'
            ).classes('text-sm text-gray-400 mb-2')

            with ui.row().classes('items-center gap-3 w-full'):
                name_input = ui.input(
                    placeholder='e.g. ROW_D_IN',
                    label='Node name',
                ).classes('flex-1')
                pose_lbl = ui.label('pose: waiting…').classes(
                    'text-xs font-mono text-gray-400')

            with ui.row().classes('items-center gap-3 mt-1'):
                cur_pill = ui.label('').classes('text-xs font-mono')
                ui.button(
                    '📍 DROP NODE', color='green',
                    on_click=lambda: self.drop_topo_node(name_input.value),
                ).classes('ml-auto').props('no-caps')

            status_lbl = ui.label('').classes('text-sm font-mono mt-1')

            def _refresh_drop() -> None:
                odom = self.latest_odom
                gps  = self.latest_gps
                if odom is not None:
                    x = odom.pose.pose.position.x
                    y = odom.pose.pose.position.y
                    gps_str = (f'  GPS({gps.latitude:.5f},{gps.longitude:.5f})'
                               if gps and gps.status.status >= 0 else '  no GPS fix')
                    pose_lbl.set_text(f'pose: ({x:.2f}, {y:.2f}){gps_str}')
                else:
                    pose_lbl.set_text('pose: waiting for /odometry/global…')

                cur = self.topo_current
                if cur and cur != '—':
                    cur_pill.set_text(f'will connect → {cur}')
                    cur_pill.style('color:#4ade80')
                else:
                    cur_pill.set_text('no current node — edge skipped')
                    cur_pill.style('color:#9ca3af')

                status_lbl.set_text(self.drop_status)
                status_lbl.style(
                    'color:#f87171' if self.drop_status.startswith('ERROR')
                    else 'color:#4ade80')

            ui.timer(0.5, _refresh_drop)

        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('GPS Map').classes('text-2xl')
            leaflet = ui.leaflet(center=(51.98278, 7.43440), zoom=16).classes('w-full h-96')
            marker  = leaflet.marker(latlng=leaflet.center)

            def update_gps_ui() -> None:
                if self.latest_gps is not None:
                    leaflet.set_center((self.latest_gps.latitude, self.latest_gps.longitude))
                    marker.move(self.latest_gps.latitude, self.latest_gps.longitude)
            ui.timer(2.0, update_gps_ui)

    def _nav_content(self) -> None:
        ui.add_head_html("""
        <style>
          .nav-card  { background:#0d1a0d; border:1px solid #166534; border-radius:8px; }
          .nav-mono  { font-family:'Courier New',monospace; }
          .nav-pill-ok   { background:#14532d; color:#4ade80; border:1px solid #166534;
                           border-radius:999px; padding:2px 10px; font-size:12px; font-weight:600; }
          .nav-pill-warn { background:#713f12; color:#fbbf24; border:1px solid #92400e;
                           border-radius:999px; padding:2px 10px; font-size:12px; font-weight:600; }
          .topo-node:hover { filter: brightness(1.4); }
        </style>
        """)

        with ui.row().classes('items-center gap-3 mb-2'):
            ui.label('Topological Navigation').classes('text-lg font-bold nav-mono')
            ui.html(f'<span class="nav-pill-{"warn" if self.topo_demo else "ok"}">'
                    f'{"DEMO MAP" if self.topo_demo else "LIVE MAP"}</span>')
            ui.html(f'<span class="nav-pill-{"ok" if _ACTION_OK else "warn"}">'
                    f'{"ACTION OK" if _ACTION_OK else "NO ACTION"}</span>')

        with ui.row().classes('w-full gap-4 items-start'):
            with ui.card().classes('flex-1 nav-card').style('min-width:0'):
                map_html = ui.html(_build_svg(self.topo_nodes, None, None))

            with ui.card().classes('nav-card').style(
                    'min-width:200px;max-width:240px;padding:16px;'
                    'gap:10px;display:flex;flex-direction:column'):

                ui.label('CURRENT NODE').classes('text-xs nav-mono').style(
                    'color:#4ade80;letter-spacing:2px')
                cur_lbl = ui.label('—').classes('text-lg font-bold nav-mono')

                ui.label('DESTINATION').classes('text-xs nav-mono mt-2').style(
                    'color:#4ade80;letter-spacing:2px')
                sel_lbl = ui.label('tap a node').classes('nav-mono').style('color:#6b7280')

                ui.label('STATUS').classes('text-xs nav-mono mt-2').style(
                    'color:#4ade80;letter-spacing:2px')
                stat_lbl = ui.label('Idle').classes('text-sm nav-mono').style('color:#d1d5db')

                ui.separator()

                go_btn   = ui.button('GO →',     color='green').classes(
                    'w-full text-lg font-bold').props('no-caps')
                stop_btn = ui.button('■ CANCEL', color='red').classes('w-full').props('no-caps')

                ui.label('ALL NODES').classes('text-xs nav-mono mt-2').style(
                    'color:#4ade80;letter-spacing:2px')
                with ui.scroll_area().style(
                        'height:160px;border:1px solid #1e3320;border-radius:4px'):
                    node_col = ui.column().style('gap:2px;padding:4px')

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
            snap = {
                'sel':   self.topo_selected,
                'cur':   self.topo_current,
                'stat':  self.topo_nav_status,
                'nav':   self.topo_navigating,
                'nodes': id(self.topo_nodes),
            }
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
                    for name in sorted(self.topo_nodes):
                        nd      = self.topo_nodes[name]
                        is_sel  = (name == self.topo_selected)
                        has_gps = bool(nd.get('meta', {}).get('gps_lat'))
                        badge   = ' 📍' if has_gps else ''
                        lat     = nd.get('meta', {}).get('gps_lat', '')
                        lon     = nd.get('meta', {}).get('gps_lon', '')
                        title   = f'{lat} {lon}'.strip()
                        style   = ('cursor:pointer;padding:4px 8px;border-radius:4px;'
                                   'font-family:monospace;font-size:12px;border:1px solid;')
                        style  += ('background:#78350f;color:#fde68a;border-color:#f59e0b'
                                   if is_sel else
                                   'background:transparent;color:#86efac;border-color:transparent')
                        n = name
                        ui.html(
                            f'<div style="{style}" title="{title}">'
                            f'{name.replace("_", " ")}{badge}</div>'
                        ).on('click', lambda _, n=n: setattr(self, 'topo_selected', n))

            cur_lbl.set_text(self.topo_current or '—')
            sel_lbl.set_text(self.topo_selected or 'tap a node')
            sel_lbl.style('color:#fde68a' if self.topo_selected else 'color:#6b7280')
            stat_lbl.set_text(self.topo_nav_status)

            can_go = (bool(self.topo_selected) and not self.topo_navigating
                      and not self.soft_estop_active)
            go_btn.set_enabled(can_go)
            stop_btn.set_enabled(self.topo_navigating)

        ui.timer(0.1, refresh_nav)
        inject_click_js()

    # ── original methods ──────────────────────────────────────────────────────

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

    def store_gps(self, msg: GPSFix) -> None:
        self.latest_gps = msg

    def store_battery(self, msg: BatteryState) -> None:
        self.latest_battery = msg

    def update_bumper_front_top(self, msg: Bool) -> None:
        self.bumper_front_top_active = msg.data

    def update_bumper_front_bottom(self, msg: Bool) -> None:
        self.bumper_front_bottom_active = msg.data

    def update_bumper_back(self, msg: Bool) -> None:
        self.bumper_back_active = msg.data

    def update_estop_front(self, msg: Bool) -> None:
        self.estop_front_active = msg.data

    def update_estop_back(self, msg: Bool) -> None:
        self.estop_back_active = msg.data


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
