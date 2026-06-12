# pylint: disable=duplicate-code
"""
Obstacle management for the Sowbot webui.

Owns the obstacle list, its YAML persistence, the /obstacles publisher,
and the UI cards rendered on the Nav and Mission tabs. Designed to be
attached to a NiceGuiNode (see ObstacleManager.attach) so it can read
the node's GPS/odom state and publish on its behalf, without inheriting
from rclpy.node.Node itself.

Threading model
---------------
self._obstacles is an immutable tuple. All writes replace it wholesale
under self._lock, and bump self._version in the same critical section.
Readers grab the tuple reference lock-free (CPython attribute reads are
atomic) and walk a consistent snapshot.

Two races are deliberately accepted:

  1. Status string: two concurrent writers can race on obstacle_status;
     last writer wins. The string is human-readable progress only, so a
     late "saved" message can briefly mask an earlier "writing…" — never
     misleading enough to matter.

  2. Topic vs file: _persist_and_publish snapshots under the lock, then
     writes the file, then publishes — both lock-free. An external topic
     observer can therefore briefly see a state newer than what's on
     disk (by milliseconds). The next save converges.

Both are commented at the relevant call sites.

Coordinate frames
-----------------
Obstacles are stored in WGS84 lat/lon — matches NavSatFix and Leaflet.
They are projected to the local map frame at publish time using the
(odom_xy ↔ gps_latlon) anchor from the host node's latest_odom and
latest_gps. Without an anchor we cannot project, so we don't publish —
a costmap subscriber would otherwise treat raw lat/lon as metric and
place obstacles thousands of kilometres from the origin.
"""

from __future__ import annotations

import math
import os
import re
import threading
from datetime import datetime, timezone
from typing import Callable, Optional

import yaml
from geometry_msgs.msg import Point32, PolygonStamped
from nicegui import ui
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                       QoSProfile, ReliabilityPolicy)

# ── Constants ────────────────────────────────────────────────────────────────

OBSTACLES_FILE = '/workspace/maps/obstacles.yaml'

_NAME_RE    = re.compile(r'^[A-Z0-9_]+$')
_NAME_CLEAN = re.compile(r'[^A-Z0-9_]')

# Latched profile — matches TMAP_QOS in ui_node so a costmap layer that
# subscribes late still receives the current obstacle set on connect.
_OBS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# WGS84 semi-major axis (m). Equirectangular projection is good to
# ~0.3% anywhere on the planet — comfortably inside metre-scale tolerance.
_R_EARTH = 6_378_137.0


# ── Geometry helpers (pure, module-level so they're easy to test) ────────────

def latlon_to_xy(lat: float, lon: float,
                 lat0: float, lon0: float) -> tuple[float, float]:
    """Convert (lat, lon) to local ENU (x, y) metres relative to (lat0, lon0)."""
    x = math.radians(lon - lon0) * _R_EARTH * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * _R_EARTH
    return x, y


def xy_to_latlon(x: float, y: float,
                 lat0: float, lon0: float) -> tuple[float, float]:
    """Inverse of latlon_to_xy."""
    lat = lat0 + math.degrees(y / _R_EARTH)
    lon = lon0 + math.degrees(x / (_R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def circle_to_ring_ll(center_lat: float, center_lon: float,
                      radius_m: float) -> list[tuple[float, float]]:
    """Tessellate a circle into a closed WGS84 ring.
    Side count scales with radius, clamped to [16, 64]."""
    n = max(16, min(64, int(radius_m * 8)))
    cos_lat = math.cos(math.radians(center_lat))
    pts: list[tuple[float, float]] = []
    for i in range(n):
        a  = 2 * math.pi * i / n
        dx = radius_m * math.cos(a)
        dy = radius_m * math.sin(a)
        lat = center_lat + math.degrees(dy / _R_EARTH)
        lon = center_lon + math.degrees(dx / (_R_EARTH * cos_lat))
        pts.append((lat, lon))
    return pts


def obstacle_to_ring_ll(obs: dict) -> list[tuple[float, float]]:
    """Single obstacle dict → closed lat/lon ring, or [] for degenerate input."""
    t = obs.get('type')
    if t == 'circle':
        c   = obs.get('center', {})
        lat = c.get('lat')
        lon = c.get('lon')
        r   = float(obs.get('radius_m', 0.0))
        if lat is None or lon is None or r <= 0:
            return []
        return circle_to_ring_ll(lat, lon, r)
    if t == 'polygon':
        ring = [(p['lat'], p['lon']) for p in obs.get('points', [])
                if 'lat' in p and 'lon' in p]
        return ring if len(ring) >= 3 else []
    return []


def obstacles_to_rings_ll(
        obstacles) -> list[list[tuple[float, float]]]:
    """All obstacles → F2C-ready list of rings. Drops degenerate entries."""
    rings = [obstacle_to_ring_ll(o) for o in obstacles]
    return [r for r in rings if len(r) >= 3]


def validate_obstacle(kind: str, lat, lon, radius_m,
                      points_ll) -> Optional[str]:
    """Validate add() inputs. Returns an error string or None.

    Module-level so tests don't have to instantiate ObstacleManager.
    """
    if kind == 'circle':
        if lat is None or lon is None:
            return 'ERROR: lat/lon required'
        try:
            lat_f, lon_f = float(lat), float(lon)
        except (TypeError, ValueError):
            return 'ERROR: lat/lon must be numeric'
        if not (math.isfinite(lat_f) and math.isfinite(lon_f)):
            return f'ERROR: lat/lon not finite ({lat}, {lon})'
        if abs(lat_f) < 1e-9 and abs(lon_f) < 1e-9:
            return 'ERROR: lat/lon are 0,0 (no fix?)'
        if radius_m <= 0:
            return 'ERROR: radius must be > 0'
        return None
    if kind == 'polygon':
        if not points_ll or len(points_ll) < 3:
            return 'ERROR: polygon needs 3+ corners'
        for la, lo in points_ll:
            if not (math.isfinite(float(la)) and math.isfinite(float(lo))):
                return f'ERROR: point not finite ({la}, {lo})'
        return None
    return f'ERROR: unknown kind {kind!r}'


# ── Manager ──────────────────────────────────────────────────────────────────

class ObstacleManager:
    """Owns the obstacle list, persistence, and the ROS publisher.

    After attach(node), the node exposes:

        node.obstacles               : tuple[dict, ...]   read-only snapshot
        node.obstacles_version       : int                bumps on every change
        node.obstacle_status         : str                last-action status
        node.default_obstacle_radius : float              shared by UI bindings
    """

    def __init__(self, path: str = OBSTACLES_FILE) -> None:
        self._path       = path
        self._lock       = threading.Lock()
        self._obstacles: tuple[dict, ...] = ()
        self._version:   int              = 0
        self._node                        = None
        self._publisher                   = None
        # _last_anchor tracks whether we've ever successfully published.
        # Used by the cold-start timer to fire exactly once when GPS+odom
        # first become available, then never again (timer cancels itself).
        self._last_anchor: Optional[tuple] = None
        self._anchor_timer                 = None

    # ── lifecycle ─────────────────────────────────────────────────────────

    def attach(self, node) -> None:
        """Wire into a NiceGuiNode. Creates publisher, kicks off background
        load, starts the cold-start anchor watcher."""
        self._node = node

        # Initialise instance attributes the UI binds to.
        node.obstacles            = ()
        node.obstacles_version    = 0
        node.obstacle_status      = ''
        if not hasattr(node, 'default_obstacle_radius'):
            node.default_obstacle_radius = 0.5

        self._publisher = node.create_publisher(
            PolygonStamped, '/obstacles', _OBS_QOS)

        threading.Thread(target=self._load, daemon=True).start()

        # Cold-start watcher: at 2 Hz, check if GPS+odom have arrived. As
        # soon as they have, publish whatever's loaded and cancel ourselves.
        # After the first publish, all subsequent updates flow through
        # _persist_and_publish, which calls _publish directly.
        self._anchor_timer = node.create_timer(
            0.5, self._cold_start_publish)

    # ── public API ────────────────────────────────────────────────────────

    def add(self, kind: str, *,
            lat: Optional[float] = None,
            lon: Optional[float] = None,
            radius_m: float = 0.5,
            points_ll: Optional[list] = None,
            name: str = '') -> Optional[str]:
        """Add a circle or polygon obstacle. Returns the allocated name on
        success, None on failure (status carries the reason either way)."""

        err = validate_obstacle(kind, lat, lon, radius_m, points_ll)
        if err:
            self._set_status(err)
            return None

        clean = _NAME_CLEAN.sub(
            '', (name or '').strip().upper().replace(' ', '_'))
        timestamp = datetime.now(timezone.utc).strftime('%d-%m-%Y_%H-%M-%S')

        # Lock covers name allocation + tuple replacement + version bump.
        # Disk write and publish happen after release, in a thread.
        with self._lock:
            existing = {o.get('name') for o in self._obstacles}
            if not clean:
                i = 1
                while f'OBS_{i}' in existing:
                    i += 1
                clean = f'OBS_{i}'
            elif not _NAME_RE.match(clean):
                self._set_status(f'ERROR: invalid name {clean!r}')
                return None
            elif clean in existing:
                self._set_status(f'ERROR: {clean} already exists')
                return None

            if kind == 'circle':
                obs = {
                    'name':       clean,
                    'type':       'circle',
                    'center':     {'lat': round(lat, 7), 'lon': round(lon, 7)},
                    'radius_m':   round(float(radius_m), 2),
                    'dropped_by': 'webui',
                    'timestamp':  timestamp,
                }
                desc = f'@ ({lat:.5f}, {lon:.5f}) r={radius_m}m'
            else:
                obs = {
                    'name':       clean,
                    'type':       'polygon',
                    'points':     [{'lat': round(la, 7), 'lon': round(lo, 7)}
                                   for la, lo in points_ll],
                    'dropped_by': 'webui',
                    'timestamp':  timestamp,
                }
                desc = f'({len(points_ll)} pts)'

            self._obstacles = self._obstacles + (obs,)
            self._version  += 1
            self._sync_node()

        self._set_status(f'{clean} {desc} — writing…')
        threading.Thread(
            target=self._persist_and_publish,
            args=(f'{clean} saved · {len(self._obstacles)} total',),
            daemon=True).start()
        return clean

    def delete(self, name: str) -> bool:
        with self._lock:
            if not any(o.get('name') == name for o in self._obstacles):
                self._set_status(f'ERROR: {name!r} not found')
                return False
            self._obstacles = tuple(o for o in self._obstacles
                                    if o.get('name') != name)
            self._version  += 1
            self._sync_node()

        self._set_status(f'deleting {name} — writing…')
        threading.Thread(
            target=self._persist_and_publish,
            args=(f'deleted {name} · {len(self._obstacles)} left',),
            daemon=True).start()
        return True

    def rings_ll(self) -> list[list[tuple[float, float]]]:
        """F2C-ready snapshot. Safe to call from any thread."""
        return obstacles_to_rings_ll(self._obstacles)

    # ── internals ─────────────────────────────────────────────────────────

    def _sync_node(self) -> None:
        """Mirror internal state onto the node. Called inside the lock."""
        if self._node is None:
            return
        self._node.obstacles         = self._obstacles
        self._node.obstacles_version = self._version

    def _set_status(self, s: str) -> None:
        """Status setter — last-writer-wins race accepted (see module docs)."""
        if self._node is not None:
            self._node.obstacle_status = s

    # ── persistence ───────────────────────────────────────────────────────

    def _load(self) -> None:
        try:
            if not os.path.exists(self._path):
                return
            with open(self._path, encoding='utf-8') as fh:
                doc = yaml.safe_load(fh) or {}
            loaded = doc.get('obstacles', []) or []
            with self._lock:
                self._obstacles = tuple(loaded)
                self._version  += 1
                self._sync_node()
            if self._node is not None:
                self._node.get_logger().info(
                    f'Loaded {len(loaded)} obstacles from {self._path}')
            # Try to publish immediately. May no-op if no GPS yet — the
            # cold-start timer will catch that case.
            self._publish()
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(
                    f'Failed to load obstacles: {e}')

    def _persist_and_publish(self, success_msg: str) -> None:
        """Atomic write (tmp + os.replace), then publish.

        Snapshot under lock, write + publish lock-free. The topic can
        briefly reflect a state newer than disk (by ms) — see module docs.
        """
        try:
            os.makedirs(os.path.dirname(self._path), exist_ok=True)
            tmp = self._path + '.tmp'
            with self._lock:
                snapshot = list(self._obstacles)
            with open(tmp, 'w', encoding='utf-8') as fh:
                yaml.dump({'obstacles': snapshot}, fh,
                          default_flow_style=False, sort_keys=False,
                          allow_unicode=True)
            os.replace(tmp, self._path)
            self._set_status(success_msg)
            if self._node is not None:
                self._node.get_logger().info(
                    f'Obstacles saved: {len(snapshot)} total')
            self._publish()
        except Exception as e:
            self._set_status(f'ERROR: {e}')
            if self._node is not None:
                self._node.get_logger().error(f'persist failed: {e}')

    # ── publishing ────────────────────────────────────────────────────────

    def _current_anchor(self) -> Optional[tuple]:
        """(map_x, map_y, lat, lon) for projecting obstacles into the map
        frame. Returns None if either odom or GPS is missing/degenerate."""
        if self._node is None:
            return None
        odom = getattr(self._node, 'latest_odom', None)
        gps  = getattr(self._node, 'latest_gps',  None)
        if odom is None or gps is None:
            return None
        lat, lon = gps.latitude, gps.longitude
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return None
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return None
        return (odom.pose.pose.position.x, odom.pose.pose.position.y,
                lat, lon)

    def _publish(self) -> None:
        """Publish each obstacle as a PolygonStamped in the map frame.
        Silent no-op if there's no projection anchor."""
        if self._publisher is None:
            return
        anchor = self._current_anchor()
        if anchor is None:
            return
        ax, ay, alat, alon = anchor

        try:
            snapshot = self._obstacles    # tuple read is atomic in CPython
            stamp    = self._node.get_clock().now().to_msg()
            for obs in snapshot:
                ring_ll = obstacle_to_ring_ll(obs)
                if not ring_ll:
                    continue
                msg = PolygonStamped()
                msg.header.stamp    = stamp
                msg.header.frame_id = 'map'
                for lat, lon in ring_ll:
                    mx, my = latlon_to_xy(lat, lon, alat, alon)
                    p = Point32()
                    p.x = float(ax + mx)
                    p.y = float(ay + my)
                    p.z = 0.0
                    msg.polygon.points.append(p)
                self._publisher.publish(msg)
            self._last_anchor = anchor
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(f'publish failed: {e}')

    def _cold_start_publish(self) -> None:
        """Run by a 2 Hz timer during cold start. Publishes once GPS+odom
        arrive, then cancels itself — all subsequent updates flow through
        _persist_and_publish."""
        if self._last_anchor is not None:
            self._cancel_anchor_timer()
            return
        if not self._obstacles:
            return   # nothing to publish yet
        if self._current_anchor() is None:
            return   # GPS / odom not ready
        self._publish()
        if self._last_anchor is not None:
            self._cancel_anchor_timer()

    def _cancel_anchor_timer(self) -> None:
        if self._anchor_timer is not None:
            try:
                self._anchor_timer.cancel()
            except Exception:
                pass
            self._anchor_timer = None


# ── UI: Nav-tab attachment ────────────────────────────────────────────────────

def attach_nav_card(node, manager: ObstacleManager) -> None:
    """Renders the 'Mark Obstacle' card in the current ui context.
    Call from inside _nav_content where you want it placed."""
    if not hasattr(node, 'default_obstacle_radius'):
        node.default_obstacle_radius = 0.5

    with ui.card().style('padding:12px 14px;flex-shrink:0;min-width:200px'):
        with ui.row().classes('items-baseline gap-2 mb-2'):
            ui.label('Mark Obstacle').classes('font-semibold')
            ui.label('circle at current GPS').classes('text-xs').style(
                'color:#8c959f')
        with ui.row().classes('items-center gap-2 w-full'):
            obs_name = ui.input(
                placeholder='blank → OBS_N', label='Name',
            ).classes('flex-1')
        with ui.row().classes('items-center gap-2 w-full mt-1'):
            ui.number(
                label='Radius', value=node.default_obstacle_radius,
                min=0.1, max=5.0, step=0.1, precision=2, suffix='m',
            ).classes('w-24').bind_value(node, 'default_obstacle_radius')

            def _mark_here() -> None:
                gps = getattr(node, 'latest_gps', None)
                if gps is None:
                    node.obstacle_status = 'ERROR: no GPS message yet'
                    return
                added = manager.add(
                    'circle', lat=gps.latitude, lon=gps.longitude,
                    radius_m=float(node.default_obstacle_radius),
                    name=obs_name.value or '')
                if added:
                    obs_name.set_value('')
                    ui.notify(
                        f'Marked {added}', type='positive', timeout=5000,
                        actions=[{'label': 'Undo',
                                  'handler': lambda: manager.delete(added)}])

            ui.button('Mark here', on_click=_mark_here).classes(
                'ml-auto').props('color=warning no-caps dense')

        status_lbl = ui.label('').classes('text-xs font-mono mt-1')

    _prev = ['']

    def _refresh() -> None:
        cur = node.obstacle_status
        if cur == _prev[0]:
            return
        _prev[0] = cur
        status_lbl.set_text(cur)
        status_lbl.style(
            'color:#cf222e' if cur.startswith('ERROR') else
            'color:#1a7f37' if cur else 'color:#57606a')

    ui.timer(0.3, _refresh)


# ── UI: Mission-tab attachment ────────────────────────────────────────────────
#
# Split in two — the controls go in the sidebar (narrow), the list panel
# goes below the map (wide). The MissionDrawHandle returned by the first
# call is passed to the second so they share in-progress drawing state and
# the click handler.

class MissionDrawHandle:
    """State + accessors shared between the sidebar controls and the list
    panel. Created by attach_mission_sidebar_controls; consumed by
    attach_mission_obstacle_panel."""

    def __init__(self, node, manager: ObstacleManager, mission_map,
                 obstacle_pad_widget) -> None:
        self.node         = node
        self.manager      = manager
        self.mission_map  = mission_map
        self.obstacle_pad = obstacle_pad_widget
        # In-progress polygon state
        self.corners:  list[tuple[float, float]] = []
        self.layer:    list = [None]
        self.markers:  list = []
        # Rendered saved-obstacle layers keyed by name → leaflet layer
        self.rendered: dict = {}
        # Current draw mode + shape — set by toggle handlers
        self.draw_mode = ['boundary']
        self.obs_shape = ['circle']

    def clear_in_progress(self) -> None:
        """Tear down any in-progress polygon. Called by the outer Clear
        button so user intent ('clear everything') actually clears."""
        self.corners.clear()
        if self.layer[0] is not None:
            try:
                self.layer[0].run_method('remove')
            except Exception:
                pass
            self.layer[0] = None
        for m in self.markers:
            try:
                m.run_method('remove')
            except Exception:
                pass
        self.markers.clear()


def attach_mission_sidebar_controls(
        node, manager: ObstacleManager, mission_map,
        boundary_click: Callable[[float, float], None]) -> MissionDrawHandle:
    """Renders the Field/Obstacle draw-mode toggle, shape sub-toggle,
    radius input, obstacle padding input, and Finish-polygon button in
    the current ui context (the Mission sidebar column).

    Also wires the mission_map's 'map-click' handler to dispatch between
    boundary-drawing (callback) and obstacle-drawing (manager.add).

    Returns a handle the caller passes to attach_mission_obstacle_panel.
    """
    if not hasattr(node, 'default_obstacle_radius'):
        node.default_obstacle_radius = 0.5

    ui.html('<div class="sec-label mt-3">Draw mode</div>')
    mode_toggle = ui.toggle(
        {'boundary': 'Field', 'obstacle': 'Obstacle'},
        value='boundary',
    ).props('dense').classes('w-full')

    with ui.column().classes('w-full gap-2 mt-2') as obs_controls:
        ui.html('<div class="sec-label">Obstacle shape</div>')
        shape_toggle = ui.toggle(
            {'circle': 'Circle', 'polygon': 'Polygon'},
            value='circle',
        ).props('dense').classes('w-full')

        ui.html('<div class="sec-label">Radius (circle)</div>')
        ui.number(value=node.default_obstacle_radius,
                  min=0.1, max=5.0, step=0.1, precision=2,
                  suffix='m').classes('w-full').bind_value(
                      node, 'default_obstacle_radius')

        finish_btn = ui.button('Finish polygon').props(
            'outline no-caps dense').classes('w-full')

    obs_controls.set_visibility(False)

    ui.html('<div class="sec-label mt-3">Obstacle padding</div>')
    obstacle_pad = ui.number(
        value=0.5, min=0.0, step=0.1, precision=2, suffix='m',
    ).classes('w-full')
    ui.label('robot clearance during planning').classes(
        'text-xs').style('color:#8c959f')

    handle = MissionDrawHandle(node, manager, mission_map, obstacle_pad)

    def _on_mode(e):
        handle.draw_mode[0] = e.args
        obs_controls.set_visibility(e.args == 'obstacle')

    def _on_shape(e):
        handle.obs_shape[0] = e.args

    mode_toggle.on('update:model-value',  _on_mode)
    shape_toggle.on('update:model-value', _on_shape)

    def _redraw_in_progress() -> None:
        if handle.layer[0] is not None:
            try:
                handle.layer[0].run_method('remove')
            except Exception:
                pass
            handle.layer[0] = None
        if len(handle.corners) >= 2:
            latlngs = [[la, lo] for la, lo in handle.corners]
            handle.layer[0] = mission_map.generic_layer(
                name='polygon',
                args=[latlngs,
                      {'color': '#cf222e', 'fillOpacity': 0.15,
                       'weight': 2, 'dashArray': '4 3'}],
            )

    def _finish_polygon() -> None:
        if len(handle.corners) < 3:
            node.obstacle_status = 'ERROR: polygon needs 3+ corners'
            return
        added = manager.add('polygon', points_ll=list(handle.corners))
        # On failure (dup name etc.) keep in-progress state for retry.
        # Only tear down on success.
        if added is not None:
            handle.clear_in_progress()

    finish_btn.on_click(_finish_polygon)

    def _on_map_click(e) -> None:
        lat = e.args['latlng']['lat']
        lon = e.args['latlng']['lng']
        if handle.draw_mode[0] == 'boundary':
            boundary_click(lat, lon)
            return
        if handle.obs_shape[0] == 'circle':
            manager.add('circle', lat=lat, lon=lon,
                        radius_m=float(node.default_obstacle_radius))
            return
        # polygon: accumulate corners
        handle.corners.append((lat, lon))
        handle.markers.append(mission_map.marker(latlng=(lat, lon)))
        _redraw_in_progress()

    mission_map.on('map-click', _on_map_click)
    return handle


def attach_mission_obstacle_panel(handle: MissionDrawHandle) -> None:
    """Renders the obstacle list + Leaflet rendering panel in the current
    ui context. Call from below the map (wide layout)."""
    node        = handle.node
    manager     = handle.manager
    mission_map = handle.mission_map

    with ui.card().classes('w-full mt-3'):
        with ui.row().classes('items-baseline gap-2 mb-2'):
            ui.label('Obstacles').classes('font-semibold')
            count_lbl = ui.label('').classes('text-xs').style('color:#8c959f')
        list_col   = ui.column().style('gap:2px;width:100%')
        status_lbl = ui.label('').classes('text-xs font-mono mt-2').style(
            'color:#57606a;word-break:break-word')

    _prev = {'version': -1, 'status': None}

    def _refresh() -> None:
        v      = node.obstacles_version
        status = node.obstacle_status
        if v == _prev['version'] and status == _prev['status']:
            return
        version_changed  = v != _prev['version']
        _prev['version'] = v
        _prev['status']  = status

        status_lbl.set_text(status)
        status_lbl.style(
            'color:#cf222e' if status.startswith('ERROR') else
            'color:#1a7f37' if status else 'color:#57606a')

        if not version_changed:
            return

        obstacles = node.obstacles
        count_lbl.set_text(f'{len(obstacles)} saved')

        list_col.clear()
        if not obstacles:
            with list_col:
                ui.label('No obstacles yet').classes('text-xs').style(
                    'color:#8c959f')
        else:
            with list_col:
                for o in obstacles:
                    nm = o.get('name', '?')
                    if o.get('type') == 'circle':
                        c    = o.get('center', {})
                        desc = (f'circle r={o.get("radius_m")}m  '
                                f'({c.get("lat"):.5f}, {c.get("lon"):.5f})')
                    else:
                        desc = f'polygon ({len(o.get("points", []))} pts)'
                    with ui.row().classes('items-center gap-2 w-full'):
                        ui.label(nm).classes('text-sm font-mono').style(
                            'min-width:90px')
                        ui.label(desc).classes(
                            'text-xs font-mono flex-1').style('color:#8c959f')
                        ui.button(
                            '✕', on_click=lambda _, n=nm: manager.delete(n),
                        ).props('flat dense').classes('text-xs').style(
                            'color:#cf222e')

        # Map layers: clear-all then redraw-all. Cheap at obstacle counts < 1000.
        for lyr in handle.rendered.values():
            try:
                lyr.run_method('remove')
            except Exception:
                pass
        handle.rendered.clear()
        for o in obstacles:
            nm = o.get('name', '?')
            if o.get('type') == 'circle':
                c = o.get('center', {})
                handle.rendered[nm] = mission_map.generic_layer(
                    name='circle',
                    args=[[c.get('lat'), c.get('lon')],
                          {'radius':      float(o.get('radius_m', 0.5)),
                           'color':       '#cf222e',
                           'fillColor':   '#cf222e',
                           'fillOpacity': 0.35,
                           'weight':      2}],
                )
            else:
                latlngs = [[p['lat'], p['lon']]
                           for p in o.get('points', [])]
                handle.rendered[nm] = mission_map.generic_layer(
                    name='polygon',
                    args=[latlngs,
                          {'color':       '#cf222e',
                           'fillColor':   '#cf222e',
                           'fillOpacity': 0.35,
                           'weight':      2}],
                )

    ui.timer(0.5, _refresh)
