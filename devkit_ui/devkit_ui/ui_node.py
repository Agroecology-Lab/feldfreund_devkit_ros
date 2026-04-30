# pylint: disable=duplicate-code
"""
ui_node.py — Sowbot web cockpit on :80

Tabs
  Control   — joystick, e-stop, ESP buttons, GPS map  (original, unchanged)
  Navigate  — topological map, node picker, GO / CANCEL

Topo nav env vars
  TMAP2_FILE   path to .tmap2.yaml  (optional — falls back to built-in demo map)
"""

import os
import threading
from pathlib import Path
from typing import Optional

import yaml
import rclpy
from geometry_msgs.msg import Twist
from gps_msgs.msg import GPSFix
from nicegui import app, ui, ui_run
from nicegui.events import ClickEventArguments
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, Duration, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, String

# Action client — graceful no-op if topological_navigation_msgs not installed
_ACTION_OK = False
try:
    from rclpy.action import ActionClient
    from topological_navigation_msgs.action import GotoNode
    _ACTION_OK = True
except ImportError:
    pass

SAFETY_QOS = QoSProfile(depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        liveliness=LivelinessPolicy.AUTOMATIC,
                        liveliness_lease_duration=Duration(seconds=1))

# ── tmap2 helpers ─────────────────────────────────────────────────────────────

def _load_tmap2(path: str) -> dict[str, dict]:
    """Parse .tmap2.yaml → {name: {x, y, edges: [name, ...]}}"""
    with open(path) as f:
        raw = yaml.safe_load(f)
    nodes: dict[str, dict] = {}
    for entry in raw:
        n = entry.get('node', entry)        # handle wrapped and flat formats
        name: str = n['name']
        pos = n['pose']['position']
        edges = [e['node'] for e in n.get('edges', []) if 'node' in e]
        nodes[name] = {'x': float(pos['x']), 'y': float(pos['y']), 'edges': edges}
    return nodes


def _demo_nodes() -> dict[str, dict]:
    return {
        'DOCK':      {'x':  0.0, 'y':  0.0, 'edges': ['ROW_A_IN', 'ROW_B_IN', 'ROW_C_IN']},
        'ROW_A_IN':  {'x':  5.0, 'y':  4.0, 'edges': ['DOCK', 'ROW_A_MID']},
        'ROW_A_MID': {'x': 10.0, 'y':  4.0, 'edges': ['ROW_A_IN', 'ROW_A_END']},
        'ROW_A_END': {'x': 15.0, 'y':  4.0, 'edges': ['ROW_A_MID']},
        'ROW_B_IN':  {'x':  5.0, 'y':  0.0, 'edges': ['DOCK', 'ROW_B_MID']},
        'ROW_B_MID': {'x': 10.0, 'y':  0.0, 'edges': ['ROW_B_IN', 'ROW_B_END']},
        'ROW_B_END': {'x': 15.0, 'y':  0.0, 'edges': ['ROW_B_MID']},
        'ROW_C_IN':  {'x':  5.0, 'y': -4.0, 'edges': ['DOCK', 'ROW_C_MID']},
        'ROW_C_MID': {'x': 10.0, 'y': -4.0, 'edges': ['ROW_C_IN', 'ROW_C_END']},
        'ROW_C_END': {'x': 15.0, 'y': -4.0, 'edges': ['ROW_C_MID']},
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

    xs = [n['x'] for n in nodes.values()]
    ys = [n['y'] for n in nodes.values()]
    dx = (max(xs) - min(xs)) or 1.0
    dy = (max(ys) - min(ys)) or 1.0

    def tx(x): return _MARGIN + (x - min(xs)) / dx * (_SVG_W - 2 * _MARGIN)
    def ty(y): return _SVG_H - _MARGIN - (y - min(ys)) / dy * (_SVG_H - 2 * _MARGIN)

    parts: list[str] = [
        f'<rect width="{_SVG_W}" height="{_SVG_H}" fill="#0a160a" rx="6"/>']

    # edges
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

    # nodes + labels
    for name, nd in nodes.items():
        cx, cy = tx(nd['x']), ty(nd['y'])
        is_cur = (name == current)
        is_sel = (name == selected)

        fill   = '#16a34a' if is_cur else ('#f59e0b' if is_sel else '#162616')
        stroke = '#86efac' if is_cur else ('#fde68a' if is_sel else '#22c55e')
        sw     = 4 if (is_cur or is_sel) else 2
        lc     = '#ffffff' if is_cur else ('#fde68a' if is_sel else '#86efac')

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

        # ── original publishers / subscriptions (unchanged) ───────────────────
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.esp_enable_publisher = self.create_publisher(Empty, 'esp/enable', 1)
        self.esp_disable_publisher = self.create_publisher(Empty, 'esp/disable', 1)
        self.esp_reset_publisher = self.create_publisher(Empty, 'esp/reset', 1)
        self.esp_restart_publisher = self.create_publisher(Empty, 'esp/restart', 1)
        self.esp_configure_publisher = self.create_publisher(Empty, 'esp/configure', 1)
        self.subscription = self.create_subscription(GPSFix, 'gpsfix', self.store_gps, 1)
        self.battery_subscription = self.create_subscription(BatteryState, 'battery_state', self.store_battery, 1)
        self.bumper_front_top_subscription = self.create_subscription(Bool,
                                                                      'bumper/front_top',
                                                                      self.update_bumper_front_top,
                                                                      SAFETY_QOS)
        self.bumper_front_bottom_subscription = self.create_subscription(Bool,
                                                                         'bumper/front_bottom',
                                                                         self.update_bumper_front_bottom,
                                                                         SAFETY_QOS)
        self.bumper_back_subscription = self.create_subscription(Bool,
                                                                 'bumper/back',
                                                                 self.update_bumper_back,
                                                                 SAFETY_QOS)
        self.estop_publisher = self.create_publisher(Bool, 'estop/soft', SAFETY_QOS)
        self.estop_front_subscription = self.create_subscription(Bool,
                                                                 'estop/front',
                                                                 self.update_estop_front,
                                                                 SAFETY_QOS)
        self.estop_back_subscription = self.create_subscription(Bool,
                                                                'estop/back',
                                                                self.update_estop_back,
                                                                SAFETY_QOS)

        self.bumper_front_top_active = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active = False
        self.soft_estop_active = False
        self.estop_front_active = False
        self.estop_back_active = False
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.latest_gps = None
        self.latest_battery = None

        # ── topo nav state ────────────────────────────────────────────────────
        tmap_path = os.environ.get('TMAP2_FILE', '')
        if tmap_path and Path(tmap_path).exists():
            self.topo_nodes = _load_tmap2(tmap_path)
            self.topo_demo  = False
            self.get_logger().info(f'Loaded tmap2: {tmap_path} ({len(self.topo_nodes)} nodes)')
        else:
            self.topo_nodes = _demo_nodes()
            self.topo_demo  = True
            if tmap_path:
                self.get_logger().warn(f'TMAP2_FILE not found: {tmap_path} — using demo map')

        self.topo_selected:    Optional[str] = None
        self.topo_current:     str           = '—'
        self.topo_nav_status:  str           = 'Idle'
        self.topo_navigating:  bool          = False
        self._nav_goal_handle               = None

        self.create_subscription(String, '/current_node',
                                 lambda m: setattr(self, 'topo_current', m.data), 10)

        if _ACTION_OK:
            self._nav_ac = ActionClient(self, GotoNode, 'topological_navigation')

        # ── page ──────────────────────────────────────────────────────────────
        @ui.page('/')
        def page():
            self.content()

    # ── topo nav actions ──────────────────────────────────────────────────────

    def send_nav_goal(self, target: str) -> None:
        if not _ACTION_OK:
            self.topo_nav_status = 'ERROR: topological_navigation_msgs not installed'
            return

        # Use the real action server — fake_nav2_server handles this in sim mode
        if not self._nav_ac.wait_for_server(timeout_sec=0.0):
            self.topo_nav_status = 'ERROR: action server not available'
            return

        goal = GotoNode.Goal()
        goal.target = target
        self.topo_nav_status = f'Navigating → {target}'
        self.topo_navigating = True

        future = self._nav_ac.send_goal_async(goal, feedback_callback=self._nav_feedback)
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
        fb = feedback_msg.feedback
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
        """Original control UI — untouched."""
        with ui.row().classes('items-stretch w-[48rem] gap-3'):
            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Control').classes('text-2xl')
                ui.joystick(color='blue', size=50,
                            on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                            on_end=lambda _: self.send_speed(0.0, 0.0))
                ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')

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
                ui.button('EMERGENCY STOP', color='blue', on_click=update_button_appearance) \
                    .classes('w-40 min-h-[3rem]')

            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Data').classes('text-2xl')
                ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                slider_props = 'readonly selection-color=transparent'
                ui.slider(min=-1, max=1, step=0.05, value=0) \
                    .props(slider_props) \
                    .bind_value(self, 'linear_velocity')
                ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                ui.slider(min=-1, max=1, step=0.05, value=0) \
                    .props(slider_props) \
                    .bind_value(self, 'angular_velocity')
                ui.label('Battery').classes('text-xs mb-[-1.4em]')
                ui.label().bind_text_from(self, 'latest_battery',
                                          lambda msg: f'{msg.percentage * 100:.1f}% ({msg.voltage:.1f}V)' if msg is not None else 'N/A')

            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Safety').classes('text-2xl')
                ui.label('Bumpers').classes('text-xs mb-[-1.4em]')
                ui.label('Front Top: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'bumper_front_top_active',
                                    lambda active: 'Front Top: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('Front Bottom: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'bumper_front_bottom_active',
                                    lambda active: 'Front Bottom: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('Back: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'bumper_back_active',
                                    lambda active: 'Back: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('E-Stops').classes('text-xs mb-[-1.4em] mt-4')
                ui.label('E-Stop Front: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'estop_front_active',
                                    lambda active: 'Front: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('E-Stop Back: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'estop_back_active',
                                    lambda active: 'Back: ' + ('ACTIVE' if active else 'inactive'))

        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('ESP Control').classes('text-2xl')
            with ui.row().classes('gap-4'):
                ui.button('Enable',    color='green',  on_click=lambda: self.esp_enable_publisher.publish(Empty())).classes('w-24')
                ui.button('Disable',   color='red',    on_click=lambda: self.esp_disable_publisher.publish(Empty())).classes('w-24')
                ui.button('Reset',     color='orange', on_click=lambda: self.esp_reset_publisher.publish(Empty())).classes('w-24')
                ui.button('Restart',   color='blue',   on_click=lambda: self.esp_restart_publisher.publish(Empty())).classes('w-24')
                ui.button('Configure', color='purple', on_click=lambda: self.esp_configure_publisher.publish(Empty())).classes('w-24')

        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('GPS Map').classes('text-2xl')
            leaflet = ui.leaflet(center=(51.98278, 7.43440), zoom=16).classes('w-full h-96')
            marker = leaflet.marker(latlng=leaflet.center)

            def update_gps_ui() -> None:
                if self.latest_gps is not None:
                    leaflet.set_center((self.latest_gps.latitude, self.latest_gps.longitude))
                    marker.move(self.latest_gps.latitude, self.latest_gps.longitude)
            ui.timer(2.0, update_gps_ui)

    def _nav_content(self) -> None:
        """Topological navigation tab."""

        ui.add_head_html("""
        <style>
          .nav-card  { background:#0d1a0d; border:1px solid #166534; border-radius:8px; }
          .nav-mono  { font-family:'Courier New',monospace; }
          .nav-pill-ok   { background:#14532d; color:#4ade80; border:1px solid #166534;
                           border-radius:999px; padding:2px 10px; font-size:12px; font-weight:600; }
          .nav-pill-warn { background:#713f12; color:#fbbf24; border:1px solid #92400e;
                           border-radius:999px; padding:2px 10px; font-size:12px; font-weight:600; }
          .nav-pill-err  { background:#450a0a; color:#f87171; border:1px solid #7f1d1d;
                           border-radius:999px; padding:2px 10px; font-size:12px; font-weight:600; }
          .topo-node:hover { filter: brightness(1.4); }
        </style>
        """)

        # top status bar
        with ui.row().classes('items-center gap-3 mb-2'):
            ui.label('Topological Navigation').classes('text-lg font-bold nav-mono')
            demo_cls = 'nav-pill-warn' if self.topo_demo else 'nav-pill-ok'
            demo_txt = 'DEMO MAP' if self.topo_demo else 'LIVE MAP'
            ui.html(f'<span class="{demo_cls}">{demo_txt}</span>')
            action_cls = 'nav-pill-ok' if _ACTION_OK else 'nav-pill-warn'
            action_txt = 'ACTION OK' if _ACTION_OK else 'NO ACTION'
            ui.html(f'<span class="{action_cls}">{action_txt}</span>')

        with ui.row().classes('w-full gap-4 items-start'):

            # ── map column ────────────────────────────────────────────────────
            with ui.card().classes('flex-1 nav-card').style('min-width:0'):
                map_html = ui.html(_build_svg(self.topo_nodes, None, None))

            # ── control column ────────────────────────────────────────────────
            with ui.card().classes('nav-card').style('min-width:200px;max-width:240px;padding:16px;gap:10px;display:flex;flex-direction:column'):

                ui.label('CURRENT NODE').classes('text-xs nav-mono').style('color:#4ade80;letter-spacing:2px')
                cur_lbl = ui.label('—').classes('text-lg font-bold nav-mono')

                ui.label('DESTINATION').classes('text-xs nav-mono mt-2').style('color:#4ade80;letter-spacing:2px')
                sel_lbl = ui.label('tap a node').classes('nav-mono').style('color:#6b7280')

                ui.label('STATUS').classes('text-xs nav-mono mt-2').style('color:#4ade80;letter-spacing:2px')
                stat_lbl = ui.label('Idle').classes('text-sm nav-mono').style('color:#d1d5db')

                ui.separator()

                go_btn   = ui.button('GO →',    color='green').classes('w-full text-lg font-bold').props('no-caps')
                stop_btn = ui.button('■ CANCEL', color='red').classes('w-full').props('no-caps')

                ui.label('ALL NODES').classes('text-xs nav-mono mt-2').style('color:#4ade80;letter-spacing:2px')
                with ui.scroll_area().style('height:160px;border:1px solid #1e3320;border-radius:4px'):
                    node_col = ui.column().style('gap:2px;padding:4px')

        # ── wire up buttons ───────────────────────────────────────────────────
        def on_go() -> None:
            if self.topo_selected:
                self.send_nav_goal(self.topo_selected)

        def on_cancel() -> None:
            self.cancel_nav_goal()

        go_btn.on_click(on_go)
        stop_btn.on_click(on_cancel)

        # ── JS → Python: SVG node click ───────────────────────────────────────
        def on_node_clicked(e) -> None:
            name = (e.args or {}).get('node')
            if name and name in self.topo_nodes:
                self.topo_selected = name

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

        # ── periodic refresh (100 ms) ─────────────────────────────────────────
        _prev: dict = {}

        def refresh_nav() -> None:
            snap = {
                'sel':  self.topo_selected,
                'cur':  self.topo_current,
                'stat': self.topo_nav_status,
                'nav':  self.topo_navigating,
            }
            changed = {k for k, v in snap.items() if _prev.get(k) != v}
            if not changed:
                return
            _prev.update(snap)

            if changed & {'sel', 'cur'}:
                map_html.set_content(
                    _build_svg(self.topo_nodes, self.topo_selected, self.topo_current))
                inject_click_js()
                # rebuild node list
                node_col.clear()
                with node_col:
                    for name in sorted(self.topo_nodes):
                        is_sel = (name == self.topo_selected)
                        style = ('cursor:pointer;padding:4px 8px;border-radius:4px;'
                                 'font-family:monospace;font-size:12px;border:1px solid;')
                        style += ('background:#78350f;color:#fde68a;border-color:#f59e0b'
                                  if is_sel else
                                  'background:transparent;color:#86efac;border-color:transparent')
                        n = name
                        ui.html(f'<div style="{style}">{name.replace("_", " ")}</div>') \
                          .on('click', lambda _, n=n: setattr(self, 'topo_selected', n))

            cur_lbl.set_text(self.topo_current or '—')
            sel_lbl.set_text(self.topo_selected or 'tap a node')
            sel_lbl.style('color:#fde68a' if self.topo_selected else 'color:#6b7280')
            stat_lbl.set_text(self.topo_nav_status)

            can_go = bool(self.topo_selected) and not self.topo_navigating \
                     and not self.soft_estop_active
            go_btn.set_enabled(can_go)
            stop_btn.set_enabled(self.topo_navigating)

        ui.timer(0.1, refresh_nav)

        # first render of click handlers
        inject_click_js()

    # ── original methods (unchanged) ──────────────────────────────────────────

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


# ── entrypoints (unchanged) ───────────────────────────────────────────────────

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py,
    # but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=80)
