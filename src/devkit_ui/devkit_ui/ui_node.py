# pylint: disable=duplicate-code,too-many-lines,consider-using-with
"""
ui_node.py — Sowbot web cockpit on :80
"""

import copy
import io
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
import zipfile
from collections.abc import Callable, Iterator
from datetime import UTC, datetime
from itertools import pairwise
from operator import attrgetter
from pathlib import Path

# The following imports get generated in the Dockerfile, they aren't available to pylint
# pylint: disable=import-error
import fields2cover as f2c
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nicegui import app, ui, ui_run
from nicegui import run as ng_run
from nicegui.events import ClickEventArguments
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    Duration,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import unary_union
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import Trigger
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# MISSION: store owns missions.yaml, scheduling, and run recording.
from devkit_ui.actions import ACTIONS, action_ros_msgs
from devkit_ui.missions import MissionStore
from devkit_ui.models import (
    NodeID,
    TopoDoc,
    TopoEdge,
    TopoNode,
    TopoPose,
    TopoProperties,
    Vector2,
)

# pylint: enable=import-error
# OBSTACLE: obstacle manager + UI attachment helpers
from devkit_ui.obstacles import (
    ObstacleManager,
    attach_mission_obstacle_panel,
    attach_mission_sidebar_controls,
    attach_nav_card,
)
from devkit_ui.parse import dump_topo_yaml, parse_topo_json, parse_topo_yaml

_TOPO_SRV_OK = False
try:
    from topological_navigation_msgs.action import GotoNode
    from topological_navigation_msgs.srv import WriteTopologicalMap
    _TOPO_SRV_OK = True
except ImportError:
    pass

_ACTION_OK = False
try:
    from rclpy.action import ActionClient  # pylint: disable=ungrouped-imports
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

def _topo_to_msg(doc: TopoDoc) -> String:
    msg = String()
    msg.data = json.dumps(doc.to_dict(), ensure_ascii=False)
    return msg


def _headland_neighbour_pairs(coords: dict) -> list:
    """Given {node_name: (x, y)} for all row endpoints, return the list of
    (a, b) node-name pairs that should be joined by a headland (nav_to_pose)
    edge: each node linked only to its immediate same-end neighbour.

    Why this exists: a route between rows must hug the headland and never
    angle across a crop row. The IN/OUT label is NOT a reliable proxy for
    which physical end a node sits at — snake (boustrophedon) ordering flips
    the label↔end correspondence on alternate rows. So we classify ends by
    geometry: rows are long, so the two ends sit at the extremes of the
    row-length axis (the coordinate with the larger spread). Split nodes into
    two ends on that axis, then order each end along the cross (along-headland)
    axis and pair consecutive nodes. Chaining neighbours (never skip-linking)
    keeps every edge between physically adjacent row-ends, so A* walks the
    headland instead of cutting a chord across a row mouth.

    Used by both save_f2c_rows_to_topo (initial build) and
    repair_row_connectivity (rewire) so the two cannot drift apart. Returns an
    empty list for < 2 endpoints. Never pairs a node with itself.
    """
    pts = list(coords.items())
    if len(pts) < 2:
        return []
    xs = [p[1][0] for p in pts]
    ys = [p[1][1] for p in pts]
    end_idx   = 0 if (max(xs) - min(xs)) > (max(ys) - min(ys)) else 1
    along_idx = 1 - end_idx
    end_vals = sorted(p[1][end_idx] for p in pts)
    mid = end_vals[len(end_vals) // 2]
    end_lo = [p for p in pts if p[1][end_idx] <  mid]
    end_hi = [p for p in pts if p[1][end_idx] >= mid]
    out: list = []
    for group in (end_lo, end_hi):
        group.sort(key=lambda p: p[1][along_idx])
        for (a_name, _), (b_name, _) in pairwise(group):
            if a_name != b_name:
                out.append((a_name, b_name))
    return out
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

def _demo_doc() -> TopoDoc:
    return TopoDoc(
        name="mixed_test_map",
        nodes=[
            TopoNode(name='N1', pose=TopoPose(x=0.0, y=0.0), edges=['N2'], meta={}),
            TopoNode(name='N2', pose=TopoPose(x=3.0, y=0.0), edges=['N1', 'N3'], meta={}),
            TopoNode(name='N3', pose=TopoPose(x=6.0, y=0.0), edges=['N2', 'N4'], meta={}),
            TopoNode(name='N4', pose=TopoPose(x=9.0, y=0.0), edges=['N3', 'N5'], meta={}),
            TopoNode(name='N5', pose=TopoPose(x=12.0, y=0.0), edges=['N4', 'N6'], meta={}),
            TopoNode(name='N6', pose=TopoPose(x=15.0, y=0.0), edges=['N5'], meta={}),
        ],
    )


# ── SVG renderer ──────────────────────────────────────────────────────────────

_SVG_W, _SVG_H = 900, 480
_MARGIN, _NODE_R = 56, 17
_TF_STALENESS_LIMIT = 2.0  # s — map->base_link older than this: don't draw it


def _node_transform(nodes: Iterator[TopoNode]):
    """World->screen transform (tx, ty) + extents (dx, dy) for the node bbox.

    Shared by _build_svg and _build_robot_svg so both layers map coordinates
    identically.
    """
    xs, ys = [], []
    for node in nodes:
        xs.append(node.x)
        ys.append(node.y)
    dx = (max(xs) - min(xs)) or 1.0
    dy = (max(ys) - min(ys)) or 1.0
    x_min, y_min = min(xs), min(ys)

    def tx(x):
        return _MARGIN + (x - x_min) / dx * (_SVG_W - 2 * _MARGIN)
    def ty(y):
        return _SVG_H - _MARGIN - (y - y_min) / dy * (_SVG_H - 2 * _MARGIN)
    return tx, ty, dx, dy


def _build_robot_svg(nodes: Iterator[TopoNode], robot: tuple | None) -> str:
    """Transparent overlay SVG with only the live robot marker (map frame).

    Kept separate from _build_svg so robot movement updates this layer alone,
    never replacing the clickable node DOM (which would drop its click handlers).
    """
    empty = (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
             f'xmlns="http://www.w3.org/2000/svg"></svg>')
    if not nodes or robot is None:
        return empty
    tx, ty, dx, dy = _node_transform(nodes)
    rx, ry, ryaw = robot
    mx, my = tx(rx), ty(ry)
    al = 0.07 * max(dx, dy)
    ex, ey = tx(rx + al * math.cos(ryaw)), ty(ry + al * math.sin(ryaw))
    return (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg">'
            f'<line x1="{mx:.1f}" y1="{my:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" '
            f'stroke="#cf222e" stroke-width="3" stroke-linecap="round"/>'
            f'<circle cx="{mx:.1f}" cy="{my:.1f}" r="7" fill="#cf222e" '
            f'stroke="#ffffff" stroke-width="2"/></svg>')


def _build_svg(doc: TopoDoc, selected: str | None, current: str | None) -> str:
    if not doc.nodes:
        return (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
                f'style="background:#f6f8fa;border-radius:4px;border:1px solid #d0d7de">'
                f'<text x="{_SVG_W//2}" y="{_SVG_H//2}" text-anchor="middle" '
                f'fill="#8c959f" font-family="Courier New" font-size="13">No map loaded</text>'
                f'</svg>')

    tx, ty, _dx, _dy = _node_transform(doc.nodes)

    parts: list[str] = [f'<rect width="{_SVG_W}" height="{_SVG_H}" fill="#f6f8fa" rx="4"/>']

    drawn: set = set()
    for nd in doc.nodes:
        for tgt in nd.edges:
            tgt_name = tgt.node
            key = tuple(sorted([nd.name, tgt_name]))
            if key in drawn or not doc.has_node(tgt_name):
                continue
            drawn.add(key)

            tgt_node = doc.get_node(tgt_name)

            is_row_edge = (nd.meta.get('row_id') is not None
                           or tgt_node.meta.get('row_id') is not None)
            parts.append(
                f'<line x1="{tx(nd.x):.1f}" y1="{ty(nd.y):.1f}" '
                f'x2="{tx(tgt_node.x):.1f}" y2="{ty(tgt_node.y):.1f}" '
                f'stroke="{"#d8e8fd" if is_row_edge else "#d0d7de"}" '
                f'stroke-width="{"2" if is_row_edge else "1"}" stroke-linecap="round"/>')

    for nd in doc.nodes:
        name = nd.name
        cx, cy = tx(nd.x), ty(nd.y)
        is_cur = name == current
        is_sel = name == selected
        is_row = nd.meta.get('row_id') is not None

        # pylint: disable=multiple-statements
        if is_cur:
            fill, stroke, sw = '#dafbe1', '#1a7f37', 2
        elif is_sel:
            fill, stroke, sw = '#fff8c5', '#9a6700', 2
        elif is_row:
            fill, stroke, sw = '#d8e8fd', '#0969da', 1
        elif nd.meta.get('dropped_by'):
            fill, stroke, sw = '#f6f8fa', '#8c959f', 1
        else:
            fill, stroke, sw = '#ffffff', '#d0d7de', 1
        # pylint: enable=multiple-statements

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
                f'{nd.meta.get("row_role","?")[0].upper()}{nd.meta.get("row_id","")}'
                f'</text>')

        parts.append(
            f'<text x="{cx:.1f}" y="{cy+_NODE_R+12:.1f}" text-anchor="middle" '
            f'fill="{label_col}" font-family="Courier New" font-size="9" '
            f'style="pointer-events:none">{name}</text>')

    return (f'<svg id="topo-map" width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg">{"".join(parts)}</svg>')


# ── Fields2Cover geometry helpers ─────────────────────────────────────────────

def _f2c_latlon_to_xy(lat: float, lon: float,
                      lat0: float, lon0: float) -> tuple[float, float]:
    R = 6_378_137.0
    x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R
    return x, y


def _f2c_xy_to_latlon(x: float, y: float,
                      lat0: float, lon0: float) -> tuple[float, float]:
    R   = 6_378_137.0
    lat = lat0 + math.degrees(y / R)
    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
    return lat, lon


# OBSTACLE: shapely post-clipping for swath/obstacle avoidance.
#
# We do not trust F2C's interior-ring handling. Across F2C builds and
# Python-binding versions the behaviour of SG_BruteForce with respect to
# Cell holes is inconsistent — sometimes swaths are clipped against
# holes, sometimes they aren't, with no error either way. Field tests
# showed swaths running straight through marked obstacles even with CW-
# wound interior rings.
#
# The fix: generate swaths against the outer boundary (F2C's job), then
# compute swath_line.difference(union_of_obstacle_polygons) ourselves
# (shapely's job). This is observable, deterministic, and guarantees the
# lines you see on the Mission map are the lines the robot will follow.
#
# Obstacle rings are still added to the F2C Cell as a hint — belt and
# braces — but the safety net is the shapely post-clip.
#
# HEADLAND: optional Minkowski erosion of the field by headland_width_m
# before swath generation, so swaths don't start/end exactly at the
# field boundary. Done via F2C's HG_Const_gen; wrapped in try/except
# because the headland call can fail on degenerate / very narrow fields.
#
# SNAKE: optional boustrophedon ordering — every second swath reversed
# so end-of-swath-N is near start-of-swath-N+1. Done in Python rather
# than via f2c.RP_Snake to avoid depending on which route-planner API
# the local F2C build exposes. Snake-flip happens *before* shapely
# clipping so direction is preserved per-fragment when an obstacle
# splits a swath into pieces.
def _run_f2c(corners_ll: list,
             obstacle_rings: list,
             tool_width: float,
             angle_deg: float,
             obstacle_pad_m: float = 0.0,
             headland_width_m: float = 0.0,
             snake_order: bool = True) -> list:

    def _log(msg):
        print(f'[F2C] {msg}', file=sys.stderr, flush=True)

    _log(f'called: {len(corners_ll)} boundary pts, '
         f'{len(obstacle_rings)} obstacles, pad={obstacle_pad_m}m, '
         f'headland={headland_width_m}m, snake={snake_order}, '
         f'width={tool_width}m, angle={angle_deg}°')

    try:
        has_shapely = True
        _log('shapely OK')
    except ImportError as e:
        has_shapely = False
        _log(f'shapely MISSING: {e}')

    lat0, lon0 = corners_ll[0]

    # ── Outer boundary ────────────────────────────────────────────────
    outer = f2c.LinearRing()
    for lat, lon in corners_ll:
        x, y = _f2c_latlon_to_xy(lat, lon, lat0, lon0)
        outer.addPoint(f2c.Point(x, y, 0))
    outer.closeRing()
    cell = f2c.Cell()
    cell.addRing(outer)

    # ── Obstacles: shapely polys for post-clip + F2C hole hints ──────
    obstacle_polys_xy: list = []
    for idx, ring_ll in enumerate(obstacle_rings):
        if len(ring_ll) < 3:
            _log(f'obstacle {idx}: skipped (only {len(ring_ll)} pts)')
            continue
        pts_xy = [_f2c_latlon_to_xy(lat, lon, lat0, lon0)
                  for lat, lon in ring_ll]
        _log(f'obstacle {idx}: {len(pts_xy)} pts, '
             f'xy bbox=({min(p[0] for p in pts_xy):.1f},{min(p[1] for p in pts_xy):.1f}) '
             f'to ({max(p[0] for p in pts_xy):.1f},{max(p[1] for p in pts_xy):.1f})')

        if has_shapely:
            poly = Polygon(pts_xy)
            _log(f'obstacle {idx}: shapely poly valid={poly.is_valid} '
                 f'empty={poly.is_empty} area={poly.area:.3f}m²')
            if obstacle_pad_m > 0:
                poly = poly.buffer(obstacle_pad_m, join_style=2, resolution=8)
                _log(f'obstacle {idx}: after buffer({obstacle_pad_m}m) '
                     f'empty={poly.is_empty} type={poly.geom_type} '
                     f'area={poly.area:.3f}m²')
            if poly.is_empty or poly.geom_type != 'Polygon':
                _log(f'obstacle {idx}: DROPPED (empty or non-Polygon)')
                continue
            obstacle_polys_xy.append(poly)
            pts_xy = list(poly.exterior.coords)

        hole = f2c.LinearRing()
        for x, y in reversed(pts_xy):
            hole.addPoint(f2c.Point(x, y, 0))
        hole.closeRing()
        cell.addRing(hole)

    _log(f'built cell with {len(obstacle_polys_xy)} shapely obstacles')

    # ── Headland inset ───────────────────────────────────────────────
    # Shrinks the cover area by headland_width_m on all sides so the
    # robot has room to turn at field edges instead of starting/ending
    # swaths at the boundary itself. Skipped when 0 — preserves the
    # original behaviour for backwards compatibility with saved fields.
    swath_cell = cell
    if headland_width_m > 0:
        try:
            hg = f2c.HG_Const_gen()
            inner = hg.generateHeadlands(f2c.Cells(cell), headland_width_m)
            if inner.size() > 0:
                swath_cell = inner.getGeometry(0)
                _log(f'headland: inset {headland_width_m}m → '
                     f'{inner.size()} sub-cell(s)')
            else:
                _log(f'headland: inset {headland_width_m}m produced 0 '
                     f'cells (too wide?) — using full boundary')
        except Exception as e:
            _log(f'headland generation failed: {e} — using full boundary')

    # ── Swath generation ─────────────────────────────────────────────
    angle_rad = math.radians(angle_deg % 180)
    sg     = f2c.SG_BruteForce()
    swaths = sg.generateSwaths(angle_rad, tool_width, swath_cell)

    raw_xy: list = []
    for i in range(swaths.size()):
        path = swaths.at(i).getPath()
        pts  = []
        for j in range(path.size()):
            pt = path.getGeometry(j)
            pts.append((pt.getX(), pt.getY()))
        if len(pts) >= 2:
            raw_xy.append(pts)
    _log(f'F2C produced {len(raw_xy)} raw swaths')

    # ── Snake ordering (Python-side boustrophedon) ───────────────────
    # F2C's BruteForce returns swaths spatially sorted along the
    # perpendicular to `angle`. Reversing every other one means the end
    # of swath N is near the start of swath N+1 — minimising inter-row
    # travel and giving the topo graph a natural chain order.
    #
    # Done before shapely clipping so direction is preserved when an
    # obstacle splits a swath into multiple fragments.
    if snake_order and raw_xy:
        raw_xy = [list(reversed(pts)) if i % 2 == 1 else list(pts)
                  for i, pts in enumerate(raw_xy)]
        _log(f'snake-flipped {len(raw_xy)} swaths')

    # ── Post-clip swaths against obstacle union ──────────────────────
    if has_shapely and obstacle_polys_xy:
        obstacles_union = unary_union(obstacle_polys_xy)
        _log(f'obstacle union: type={obstacles_union.geom_type} '
             f'area={obstacles_union.area:.3f}m² '
             f'bounds={obstacles_union.bounds}')
        if raw_xy:
            sample = raw_xy[0]
            _log(f'first swath: {len(sample)} pts, '
                 f'from ({sample[0][0]:.1f},{sample[0][1]:.1f}) '
                 f'to ({sample[-1][0]:.1f},{sample[-1][1]:.1f})')
        clipped: list = []
        clipped_count, dropped_count = 0, 0
        for pts in raw_xy:
            line = LineString(pts)
            intersects = line.intersects(obstacles_union)
            remaining = line.difference(obstacles_union)
            if remaining.is_empty:
                dropped_count += 1
                continue
            geoms = (list(remaining.geoms)
                     if isinstance(remaining, MultiLineString)
                     else [remaining])
            for sub in geoms:
                if sub.length > tool_width * 0.5:
                    clipped.append(list(sub.coords))
                    if intersects:
                        clipped_count += 1
        _log(f'clipped {clipped_count} swaths against obstacles, '
             f'{dropped_count} fully dropped, final={len(clipped)}')
        raw_xy = clipped
    else:
        _log(f'NO CLIPPING: has_shapely={has_shapely}, '
             f'obstacle_polys_xy={len(obstacle_polys_xy)}')

    # ── Project back to lat/lon ──────────────────────────────────────
    result: list = []
    for pts_xy in raw_xy:
        pts_ll = [_f2c_xy_to_latlon(x, y, lat0, lon0) for x, y in pts_xy]
        if len(pts_ll) >= 2:
            result.append(pts_ll)
    _log(f'returning {len(result)} swaths to UI')
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
        self.create_subscription(NavSatFix,    '/gnss/fix',           self.store_gps,                  _SENSOR_QOS)

        # Sim GPS shim: saving a topo map hard-requires a finite, non-zero fix
        # (see save path) to anchor nodes to a datum, and at cold start
        # nothing has published one yet. Gazebo's real navsat sensor IS
        # bridged onto /gnss/fix (ros_gz_bridge.yaml) — this shim used to
        # publish onto that SAME topic and rely on a discovery-time backoff
        # (get_publishers_info_by_topic) to yield to the real bridge. That
        # was racy: DDS discovery has latency, so a bridge that starts
        # publishing in the same window could be missed, letting one fake
        # fix at the hardcoded datum below reach fusioncore. That datum is
        # ~53m from a real field's actual datum (verified against
        # maps/maize_map's back-solved origin) — a jump big enough to trip
        # fusioncore's outlier gate and anchor it on the wrong reference for
        # the rest of the run, silently rejecting every subsequent real fix.
        # Fix: publish on a dedicated topic so there is no shared-topic race
        # at all, and only let the UI treat it as a real-position fallback
        # (topo-map save path) when no genuine /gnss/fix has arrived
        # recently — fusioncore never subscribes to this topic, so it can
        # no longer be corrupted by the shim regardless of timing.
        # The sim flag is the authoritative signal, plumbed from
        # manage.py's is_sim through devkit.launch.py -> ui.launch.py, so we
        # never publish this on hardware. The India datum matches the
        # leaflet centre / F2C fallback used elsewhere in this UI.
        _FAKE_GPS_TOPIC = '/gnss/fix_sim_shim'
        self.declare_parameter('sim', False)
        self._is_sim = bool(self.get_parameter('sim').value)
        self._FAKE_GPS_LAT = 9.045094
        self._FAKE_GPS_LON = 77.792024
        self._FAKE_GPS_ALT = 40.0
        # Sentinel marking our own synthetic fixes. Kept even though the
        # shim is off /gnss/fix now: store_fake_gps still uses it to make
        # sure we're not somehow processing our own echo, and it's cheap
        # insurance against a future re-merge of the two topics.
        # status.service is uint16 and real receivers only set the low bits
        # (GPS=1/GLONASS=2/COMPASS=4/GALILEO=8, max 15), so a high value is
        # unambiguous and assignable.
        self._FAKE_GPS_SENTINEL = 0xF000
        self._last_real_gps_t = 0.0
        if self._is_sim:
            self._fake_gps_pub = self.create_publisher(
                NavSatFix, _FAKE_GPS_TOPIC, _SENSOR_QOS)
            self.create_subscription(
                NavSatFix, _FAKE_GPS_TOPIC, self.store_fake_gps, _SENSOR_QOS)
            self.create_timer(1.0, self._publish_fake_gps)
            self.get_logger().info(
                f'Sim mode: publishing fake fix on {_FAKE_GPS_TOPIC} at datum '
                f'({self._FAKE_GPS_LAT}, {self._FAKE_GPS_LON}) — fusioncore '
                'does not subscribe to this topic')
        self.create_subscription(BatteryState, 'battery_state',       self.store_battery,               1)
        self.create_subscription(Bool,         'bumper/front_top',    self.update_bumper_front_top,    SAFETY_QOS)
        self.create_subscription(Bool,         'bumper/front_bottom', self.update_bumper_front_bottom, SAFETY_QOS)
        self.create_subscription(Bool,         'bumper/back',         self.update_bumper_back,         SAFETY_QOS)
        self.create_subscription(Bool,         'estop/front',         self.update_estop_front,         SAFETY_QOS)
        self.create_subscription(Bool,         'estop/back',          self.update_estop_back,          SAFETY_QOS)

        _ODOM_QOS = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._fusion_odom_seen: bool = False

        # Position covariance (diagonal xx) threshold below which a
        # /fusion/odom message is trusted enough to take over from ground
        # truth. fusioncore publishes early, low-confidence estimates before
        # heading validates / lever arm resolves (e.g. covariance still huge,
        # origin at 0,0) — latching onto the FIRST message unconditionally
        # (previous behaviour) froze the UI marker on a garbage pose forever,
        # since /odom stops updating latest_odom the instant any /fusion/odom
        # message arrives. Now we keep tracking /odom until fusion's own
        # reported covariance says it's actually trustworthy.
        #
        # Covariance alone is not enough: a UKF anchored to a degenerate GNSS
        # origin (e.g. the world had no <spherical_coordinates>, so every fix
        # was frozen at lat=0/lon=0) can report LOW covariance while dead
        # reckoning off pure IMU+encoder with zero real GNSS correction —
        # confidently wrong, not uncertain. Low covariance only means "the
        # filter is internally consistent", not "the filter is right". So
        # also require a real GNSS fix within the last few seconds
        # (self._last_real_gps_t, set in store_gps and already used to gate
        # the fake-fix shim) before trusting /fusion/odom at all. This is
        # belt-and-suspenders on top of fixing the actual root cause (missing
        # spherical_coordinates in the generated world) — it stops the UI
        # from silently re-trusting a confidently-wrong fusion pose if that
        # world-georeference patch ever regresses again.
        _FUSION_COV_TRUST_THRESHOLD = 1.0  # m^2 — matches fusioncore_sim.yaml's loosened floor
        _FUSION_GNSS_STALENESS_LIMIT = 5.0  # s — real /gnss/fix must be this fresh

        def _store_fusion_odom(m: Odometry) -> None:
            cov_xx = m.pose.covariance[0]
            if cov_xx <= 0.0 or cov_xx > _FUSION_COV_TRUST_THRESHOLD:
                return  # not trustworthy yet — let /odom keep driving the marker
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_real_gps_t > _FUSION_GNSS_STALENESS_LIMIT:
                return  # low covariance but no recent real GNSS correction —
                        # confidently wrong, not confidently right
            self._fusion_odom_seen = True
            self.latest_odom = m

        self.create_subscription(Odometry, '/fusion/odom', _store_fusion_odom, _ODOM_QOS)
        self.create_subscription(Odometry, '/odom',
                                 self._odom_fallback, _ODOM_QOS)

        # TF: _robot_pose() needs the actual map->base_link transform, not a
        # raw odom-frame pose. odom frame origin is wherever the robot
        # started dead-reckoning (spawn point in sim) — it does NOT coincide
        # with map (0,0), so plotting raw /odom against topo_nodes (map
        # frame) puts the marker off wherever it actually is, potentially
        # off-canvas entirely. Buffer/listener give us a real map->base_link
        # lookup regardless of whether map->odom is a static bootstrap
        # transform (sim) or a live localisation output (real hardware).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # /odometry/global is fed by a relay of /fusion/odom in sim (see
        # sim_nav.launch.py) — same trust gating applies via _odom_fallback's
        # self._fusion_odom_seen check, so it won't overwrite a good pose with
        # a stale/uninitialized one either.
        self.create_subscription(Odometry, '/odometry/global',
                                 self._odom_fallback, _ODOM_QOS)

        self.create_subscription(String, '/current_node',
                                 lambda m: setattr(self, 'topo_current', m.data), _SENSOR_QOS)

        self._topo_doc:  TopoDoc | None = _demo_doc()
        self._topo_demo: bool           = False
        self.create_subscription(String, '/topological_map_2', self._on_topo_map, TMAP_QOS)
        self._topo_map_pub = self.create_publisher(String, '/topological_map_2', TMAP_QOS)

        if _TOPO_SRV_OK:
            self._write_map_cli  = self.create_client(WriteTopologicalMap,
                '/topological_map_manager2/write_topological_map')
            self._switch_map_cli = self.create_client(WriteTopologicalMap,
                '/topological_map_manager2/switch_topological_map')

        if _ACTION_OK:
            self._nav_ac = ActionClient(self, GotoNode, 'topological_navigation')

        # Row discovery: idle service clients + live status feed from
        # row_discovery_node (started alongside limbic_row_follow in
        # sim_nav.launch.py / row_follow.launch.py). Node may not exist if
        # the launch file hasn't been updated yet -- wait_for_service in
        # start_discovery()/stop_discovery() surfaces that as a status
        # string rather than raising.
        self._row_discovery_start_cli = self.create_client(
            Trigger, '/row_discovery_node/start_discovery')
        self._row_discovery_stop_cli = self.create_client(
            Trigger, '/row_discovery_node/stop_discovery')
        self.create_subscription(String, '/row_discovery/status',
            lambda m: setattr(self, 'discovery_status', m.data), _SENSOR_QOS)
        self.discovery_active: bool = False
        self.discovery_status: str  = 'idle'

        self.latest_odom:    Odometry | None     = None
        self.latest_gps:     NavSatFix | None    = None
        self.latest_battery: BatteryState | None = None

        self.bumper_front_top_active    = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active         = False
        self.soft_estop_active          = False
        self.estop_front_active         = False
        self.estop_back_active          = False
        self.linear_velocity            = 0.0
        self.angular_velocity           = 0.0

        self.topo_selected:   str    | None = None
        self.topo_current:    str           = '—'
        self.topo_nav_status: str           = 'Idle'
        self.topo_navigating: bool          = False
        self._nav_goal_handle               = None
        self.drop_status:     str           = ''

        self._track_timer:   object  | None   = None
        self._track_counter: int              = 0
        self._track_prefix:  str              = ''
        self._track_row_id:  int     | None   = None
        self._track_is_row:  bool             = False
        self._track_first:   bool             = True
        self.track_status:   str              = ''

        self._f2c_swaths:     list  = []
        self._f2c_row_start:  int   = 1
        self._f2c_tool_width: float = 1.2
        self._f2c_angle_deg:  float = 0.0
        self._f2c_origin_ll = None
        self.f2c_save_status: str   = ''
        self.delete_status:   str   = ''

        # OBSTACLE: manager owns obstacles.yaml + /obstacles publisher.
        # Attach after latest_odom / latest_gps fields exist so the
        # manager can read them when projecting to the map frame.
        self._obstacle_mgr = ObstacleManager()
        self._obstacle_mgr.attach(self)

        # MISSION: store owns missions.yaml, scheduling, and run recording.
        # Attach after obstacle manager so node attributes are all present.
        self._mission_store = MissionStore()
        self._mission_store.attach(self)

        # Lazy cache of std_msgs/Bool publishers for tool topics, keyed by
        # topic name.  Created on first use by _get_tool_publisher().
        self._tool_publishers: dict = {}

        # Mission executor state.  A running mission sets _mission_running
        # True; the executor thread clears it when done (or cancelled).
        self._mission_running:   bool          = False
        self._mission_cancel:    bool          = False
        self._mission_run_id:    str | None = None   # active MissionStore id

        self._pose_fail_log_t = 0.0

        @ui.page('/')
        def page():
            self.content()

    # ── odom fallback ─────────────────────────────────────────────────────────

    def _odom_fallback(self, msg: Odometry) -> None:
        """Use /odom (wheel odometry) whenever /fusion/odom has not yet arrived.

        The original guard (if self.latest_odom is None) froze the value after
        the first message, giving a stale pose for every subsequent drop/save.
        We instead update continuously as long as /fusion/odom hasn't been seen —
        tracked by whether the subscriber lambda has ever fired (self._fusion_odom_seen).
        """
        if not self._fusion_odom_seen:
            self.latest_odom = msg

    def _robot_pose(self) -> tuple | None:
        """(x, y, yaw) of the robot in map frame, or None if unavailable.

        Real map->base_link TF lookup, not raw /odom — odom's origin is the
        robot's dead-reckoning start point (spawn in sim), not map (0,0), so
        using it raw plots the marker off by the full map->odom offset.

        Liveness and staleness are both checked against the TF result itself
        (not latest_odom, which this no longer reads) so the marker tracks
        the actual thing being drawn: if /odom dies but TF is still fresh,
        keep showing it; if TF stalls, blank it even if /odom is still
        ticking.
        """
        # TEMP DIAGNOSTIC (remove once the marker-drop cause is confirmed):
        # distinguishes "TF lookup threw" from "TF stale" from "all fine" so
        # we can see which one fires when the marker disappears on nav start.
        # Rate-limited to ~1/s so it doesn't flood the log while the failure
        # persists across many UI refresh ticks.
        now_wall = self.get_clock().now().nanoseconds * 1e-9
        can_log  = (now_wall - self._pose_fail_log_t) > 1.0

        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if can_log:
                self._pose_fail_log_t = now_wall
                self.get_logger().warn(
                    f'_robot_pose: TF lookup map->base_link failed '
                    f'({type(e).__name__}): {e}')
            return None
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > _TF_STALENESS_LIMIT:
            if can_log:
                self._pose_fail_log_t = now_wall
                self.get_logger().warn(
                    f'_robot_pose: TF map->base_link stale by '
                    f'{now - stamp:.2f}s (limit {_TF_STALENESS_LIMIT}s)')
            return None
        p, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (p.x, p.y, yaw)

    # ── map callback ──────────────────────────────────────────────────────────

    def _on_topo_map(self, msg: String) -> None:
        try:
            self._topo_doc = parse_topo_json(msg.data)
            self._topo_demo  = False
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /topological_map_2: {e}')

    # ── nav actions ───────────────────────────────────────────────────────────

    def send_nav_goal(self, target: str) -> None:
        if not _ACTION_OK:
            self.topo_nav_status = 'action unavailable (import failed)'
            return
        self.topo_nav_status = f'connecting → {target}…'
        self.topo_navigating = True
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

    def drop_topo_node(self, name: str, row_id: int | None,
                       row_role: str = 'entry') -> None:
        name = re.sub(r'[^A-Z0-9_]', '', name.strip().upper().replace(' ', '_'))
        if not name:
            self.drop_status = 'ERROR: node name required'
            return
        if not _NAME_RE.match(name):
            self.drop_status = f'ERROR: invalid name "{name}"'
            return
        if self._topo_doc.has_node(name):
            self.drop_status = f'ERROR: {name} already exists'
            return
        if self.latest_odom is None and not self._is_sim:
            self.drop_status = 'ERROR: no odometry'
            return
        if not self._topo_doc:
            self.drop_status = 'ERROR: map not loaded'
            return

        x = round(self.latest_odom.pose.pose.position.x, 3) if self.latest_odom else 0.0
        y = round(self.latest_odom.pose.pose.position.y, 3) if self.latest_odom else 0.0
        connect_to = (self.topo_current
                      if self.topo_current not in ('—', 'none', 'None', '', None) else None)
        if connect_to and not self._topo_doc.has_node(connect_to):
            connect_to = None
        if not connect_to and self.topo_selected and self._topo_doc.has_node(self.topo_selected):
            connect_to = self.topo_selected
        map_name  = self._topo_doc.name
        nav_frame = self._topo_doc.transformation.get('topo_frame_id', 'map')
        is_row    = row_id is not None

        if is_row:
            edge_action, xy_tol, yaw_tol, vert_r = _ROW_ACTION, 0.1, 0.05, 0.5
        else:
            edge_action, xy_tol, yaw_tol, vert_r = _NAV_ACTION, 0.3, 0.1,  1.0

        gps = self.latest_gps
        gps_meta: dict = {}
        if gps is not None and gps.status.status >= 0:
            gps_meta = {
                'gps_lat': round(gps.latitude, 7),
                'gps_lon': round(gps.longitude, 7),
                'gps_fix_type': int(gps.status.status),
                'gps_hdop': None,
            }

        row_meta: dict = {}
        if is_row:
            row_meta = {
                'row_id': row_id,
                'row_role': row_role,
            }

        self._topo_doc.add_node(TopoNode(
            name=name,
            nav_frame=nav_frame,
            x=x,
            y=y,
            meta={
                'map': map_name,
                'node': name,
                'pointset': map_name,
                'dropped_by': 'webui',
                'timestamp': datetime.now(UTC).strftime('%d-%m-%Y_%H-%M-%S'),
                **gps_meta,
                **row_meta
            },
            properties=TopoProperties(xy_goal_tolerance=xy_tol, yaw_goal_tolerance=yaw_tol),
            verts=[
                Vector2(x=-vert_r, y=-vert_r),
                Vector2(x=vert_r, y=-vert_r),
                Vector2(x=vert_r, y=vert_r),
                Vector2(x=-vert_r, y=vert_r),
            ],
            edges=[
                TopoEdge(action=edge_action, edge_id=f'{name}_{connect_to}', node=connect_to),
            ] if connect_to else [],
        ))

        conn_str = f' → {connect_to}' if connect_to else ''
        gps_str  = (f' [{gps_meta["gps_lat"]:.5f},{gps_meta["gps_lon"]:.5f}]'
                    if gps_meta else '')
        row_str  = f' row={row_id}/{row_role}' if is_row else ''
        self.drop_status = f'{name}{conn_str} at ({x}, {y}){row_str}{gps_str} — writing…'

        def _publish_and_persist():
            try:
                map_file      = f'/workspace/maps/{map_name}'
                installed_src = ('/workspace/install/topological_navigation/share/'
                                 'topological_navigation/config/mixed_actions_map.yaml')

                if os.path.exists(map_file):
                    file_doc = parse_topo_yaml(map_file)
                elif os.path.exists(installed_src):
                    file_doc = parse_topo_yaml(installed_src)
                    self.get_logger().info('Seeding from installed source')
                else:
                    file_doc = self._topo_doc
                    self.get_logger().warn('No YAML source — JSON fallback')

                existing_names = {e.name for e in file_doc.nodes}
                if name in existing_names:
                    self.get_logger().warn(f'Node {name} already in file — skipping write')
                    return

                dump_topo_yaml(file_doc, map_file)

                self._topo_doc = file_doc

                self.drop_status = (f'{name}{conn_str} at ({x}, {y})'
                                    f'{row_str}{gps_str} — reloading…')

                def _call(client, req, timeout=5.0):
                    ev = threading.Event()
                    res = [None]
                    def _cb(f):
                        res[0] = f.result()
                        ev.set()
                    client.call_async(req).add_done_callback(_cb)
                    ev.wait(timeout=timeout)
                    return res[0]

                if _TOPO_SRV_OK:
                    sw = WriteTopologicalMap.Request()
                    sw.filename = f'/workspace/maps/{map_name}'
                    sw.no_alias = True
                    sr = _call(self._switch_map_cli, sw)
                    if sr and sr.success:
                        self.drop_status = (f'{name}{conn_str} at ({x},{y})'
                                            f'{row_str}{gps_str} — live')
                    else:
                        self._topo_map_pub.publish(_topo_to_msg(self._topo_doc))
                        err = sr.message if sr else 'timeout'
                        self.drop_status = f'{name}{conn_str} saved (switch failed: {err})'
                        self.get_logger().warn(f'switch_topological_map failed ({err})')
                else:
                    self._topo_map_pub.publish(_topo_to_msg(self._topo_doc))
                    self.drop_status = (f'{name}{conn_str} at ({x},{y})'
                                        f'{row_str}{gps_str} — live (no srv)')
                self.get_logger().info(
                    f'Node dropped: {name} at ({x:.3f},{y:.3f}){conn_str}{row_str}{gps_str}')
            except Exception as e:
                self.drop_status = f'ERROR: {e}'
                self.get_logger().error(f'drop_topo_node failed: {e} ({type(e)}\n{traceback.format_exc()})')

        threading.Thread(target=_publish_and_persist, daemon=True).start()

    # ── track mode ────────────────────────────────────────────────────────────

    def start_track(self, prefix: str, interval: float,
                    row_id: int | None, row_role: str | None) -> None:
        prefix = re.sub(r'[^A-Z0-9_]', '', prefix.strip().upper().replace(' ', '_'))
        if not prefix:
            self.track_status = 'ERROR: prefix required'
            return
        if self._track_timer is not None:
            self.track_status = 'ERROR: already running'
            return
        existing = [n.name for n in self._topo_doc.nodes
                    if n.name.startswith(prefix + '_') and n.name[len(prefix)+1:].isdigit()]
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
            self._track_timer.cancel()
            self._track_timer = None
        if self._track_is_row and self._track_counter > 0:
            last_name = f'{self._track_prefix}_{self._track_counter}'
            self._patch_node_role(last_name, 'exit')
            self.track_status = (f'stopped — {last_name} marked exit'
                                 f'  (#{self._track_counter} nodes)')
        else:
            self.track_status = f'stopped at #{self._track_counter}'
        self._track_counter = 0
        self._track_prefix = ''
        self._track_row_id  = None
        self._track_is_row = False
        self._track_first = True

    # ── shared topo-map persistence helper ────────────────────────────────────

    # ── Row discovery ────────────────────────────────────────────────────────

    def start_discovery(self) -> None:
        """Call row_discovery_node's start_discovery Trigger service.

        NOTE: this only gates the UI confirmation flow (see _nav_content) --
        it is not itself a safety interlock. There is no person-detection
        system wired into this codebase to check against yet; the operator
        confirmation dialog is the only guard that exists right now.
        """
        self.discovery_status = 'starting…'
        def _work():
            if not self._row_discovery_start_cli.wait_for_service(timeout_sec=2.0):
                self.discovery_active = False
                self.discovery_status = 'ERROR: row_discovery_node not running'
                return
            def _cb(f):
                try:
                    res = f.result()
                    self.discovery_active = res.success
                    self.discovery_status = res.message or (
                        'running' if res.success else 'failed to start')
                except Exception as e:
                    self.discovery_active = False
                    self.discovery_status = f'ERROR: {e}'
            self._row_discovery_start_cli.call_async(
                Trigger.Request()).add_done_callback(_cb)
        threading.Thread(target=_work, daemon=True).start()

    def stop_discovery(self) -> None:
        def _work():
            def _cb(f):
                try:
                    res = f.result()
                    self.discovery_status = res.message or 'stopped'
                except Exception as e:
                    self.discovery_status = f'ERROR: {e}'
                finally:
                    self.discovery_active = False
            self._row_discovery_stop_cli.call_async(
                Trigger.Request()).add_done_callback(_cb)
        threading.Thread(target=_work, daemon=True).start()

    def _persist_and_reload(self, modify_fn: Callable[[TopoDoc], None], status_attr: str,
                             success_msg: str) -> None:
        map_name = self._topo_doc.name
        map_file = f'/workspace/maps/{map_name}'
        installed_src = ('/workspace/install/topological_navigation/share/'
                         'topological_navigation/config/mixed_actions_map.yaml')

        def _work():
            try:
                if os.path.exists(map_file):
                    file_doc = parse_topo_yaml(map_file)
                elif os.path.exists(installed_src):
                    file_doc = parse_topo_yaml(installed_src)
                else:
                    file_doc = copy.deepcopy(self._topo_doc)

                # Backfill missing per-node entry meta (hand-written nodes).
                file_doc.ensure_meta(map_name)

                modify_fn(file_doc)

                dump_topo_yaml(file_doc, map_file)
                self._topo_doc = file_doc

                def _call(client, req, timeout=5.0):
                    ev = threading.Event()
                    res = [None]
                    def _cb(f):
                        res[0] = f.result()
                        ev.set()
                    client.call_async(req).add_done_callback(_cb)
                    ev.wait(timeout=timeout)
                    return res[0]

                if _TOPO_SRV_OK:
                    sw = WriteTopologicalMap.Request()
                    sw.filename = map_file
                    sw.no_alias = True
                    sr = _call(self._switch_map_cli, sw)
                    if sr and sr.success:
                        setattr(self, status_attr, f'{success_msg} — live')
                    else:
                        self._topo_map_pub.publish(_topo_to_msg(self._topo_doc))
                        err = sr.message if sr else 'timeout'
                        setattr(self, status_attr,
                                f'{success_msg} (switch failed: {err})')
                else:
                    self._topo_map_pub.publish(_topo_to_msg(self._topo_doc))
                    setattr(self, status_attr, f'{success_msg} — live (no srv)')
                self.get_logger().info(f'_persist_and_reload: {success_msg}')
            except Exception as e:
                setattr(self, status_attr, f'ERROR: {e}')
                self.get_logger().error(f'_persist_and_reload failed: {e} ({type(e)}\n{traceback.format_exc()})')

        threading.Thread(target=_work, daemon=True).start()

    # ── F2C → topo rows ──────────────────────────────────────────────────────

    def save_f2c_rows_to_topo(self, prefix: str, row_id_start: int,
                              overwrite: bool = False) -> None:
        prefix = re.sub(r'[^A-Z0-9_]', '',
                        (prefix or '').strip().upper().replace(' ', '_'))
        if not prefix:
            self.f2c_save_status = 'ERROR: prefix required'
            return
        if not self._f2c_swaths:
            self.f2c_save_status = 'ERROR: no rows planned — click Plan Rows first'
            return
        # In sim mode anchor_x/y are always 0.0 (see below), so latest_odom is
        # not actually used — skip the guard to avoid a false "no odometry" error
        # before Gazebo is launched.
        if self.latest_odom is None and not self._is_sim:
            self.f2c_save_status = 'ERROR: no odometry'
            return
        if self.latest_gps is None:
            self.f2c_save_status = 'ERROR: no GPS fix yet (/gnss/fix or sim shim)'
            return

        _lat = self.latest_gps.latitude
        _lon = self.latest_gps.longitude
        _status = self.latest_gps.status.status
        if not (math.isfinite(_lat) and math.isfinite(_lon)):
            self.f2c_save_status = (
                f'ERROR: GPS lat/lon not finite ({_lat}, {_lon}) status={_status}')
            return
        if abs(_lat) < 1e-9 and abs(_lon) < 1e-9:
            self.f2c_save_status = (
                f'ERROR: GPS lat/lon are 0,0 — no fix yet (status={_status})')
            return
        if _status < 0:
            self.get_logger().warn(
                f'save_f2c_rows: proceeding with status={_status} '
                f'(lat={_lat:.7f}, lon={_lon:.7f})')

        if not self._topo_doc:
            self.f2c_save_status = 'ERROR: map not loaded'
            return

        anchor_x   = 0.0 if self._is_sim else self.latest_odom.pose.pose.position.x
        anchor_y   = 0.0 if self._is_sim else self.latest_odom.pose.pose.position.y
        # Anchor for the lat/lon -> local xy conversion. On real hardware
        # latest_gps IS the survey origin and is correct. In sim, latest_gps is
        # the static datum fix, which is generally NOT where the F2C field was
        # drawn — anchoring to it offsets every row by the field-to-datum
        # distance (the "robot drove to India / off the map" bug). Anchoring to
        # the field's own reference corner instead makes the round-trip cancel,
        # so rows land at the local odom origin like get_maize_topo.py output.
        if self._is_sim and self._f2c_origin_ll is not None:
            anchor_lat, anchor_lon = self._f2c_origin_ll
        elif self._is_sim:
            self.f2c_save_status = (
                'ERROR: no F2C field origin in memory (re-run "Plan Rows" '
                'first) — saving now would anchor rows to the sim GPS '
                'datum instead of the field, offsetting every node.')
            return
        else:
            anchor_lat = self.latest_gps.latitude
            anchor_lon = self.latest_gps.longitude
        fix_type   = int(self.latest_gps.status.status)

        map_name  = self._topo_doc.name or 'mixed_test_map'
        nav_frame = self._topo_doc.transformation.get('topo_frame_id', 'map')
        timestamp = datetime.now(UTC).strftime('%d-%m-%Y_%H-%M-%S')

        connect_to = (self.topo_current
                      if self.topo_current not in ('—', 'none', 'None', '', None)
                      else None)
        if connect_to and not self._topo_doc.has_node(connect_to):
            connect_to = None
        if not connect_to and self.topo_selected and self._topo_doc.has_node(self.topo_selected):
            connect_to = self.topo_selected

        new_topo_nodes: dict[NodeID, TopoNode] = {}

        added: list[int] = []
        row_names: dict[int, tuple[NodeID, NodeID]] = {}
        row_coords: dict[str, tuple[float, float]] = {}   # node_name -> (x, y) for headland end-classification
        skipped: list[int] = []

        verts = [Vector2(x=-0.5, y=-0.5), Vector2(x= 0.5, y=-0.5),
                 Vector2(x= 0.5, y= 0.5), Vector2(x=-0.5, y= 0.5)]

        def _disk_node(name, x, y, role, lat, lon, edges, rid) -> TopoNode:
            return TopoNode(
                name=name,
                nav_frame=nav_frame,
                edges=edges,
                pose=TopoPose(x=x, y=y),
                properties=TopoProperties(
                    xy_goal_tolerance=0.1,
                    yaw_goal_tolerance=0.05,
                    dropped_by='webui_f2c',
                    timestamp=timestamp,
                    gps_lat=round(lat, 7),
                    gps_lon=round(lon, 7),
                    gps_fix_type=fix_type,
                    gps_hdop=None,
                    row_id=rid,
                    row_role=role,
                ),
                verts=verts,
                meta={'map': map_name, 'node': name}
            )

        for i, swath in enumerate(self._f2c_swaths):
            if len(swath) < 2:
                continue
            rid = row_id_start + i
            in_lat,  in_lon  = swath[0]
            out_lat, out_lon = swath[-1]

            ie, in_n = _f2c_latlon_to_xy(in_lat,  in_lon,  anchor_lat, anchor_lon)
            oe, on_  = _f2c_latlon_to_xy(out_lat, out_lon, anchor_lat, anchor_lon)
            ix, iy = round(anchor_x + ie, 3), round(anchor_y + in_n, 3)
            ox, oy = round(anchor_x + oe, 3), round(anchor_y + on_, 3)

            in_name  = f'{prefix}_R{rid}_IN'
            out_name = f'{prefix}_R{rid}_OUT'
            if not overwrite and (in_name in new_topo_nodes or out_name in new_topo_nodes):
                skipped.append(rid)
                continue

            ui_meta_common = {'dropped_by': 'webui_f2c', 'timestamp': timestamp,
                              'gps_fix_type': fix_type, 'gps_hdop': None,
                              'row_id': rid}

            in_edges = [TopoEdge(action=_ROW_ACTION, edge_id=f'{in_name}_{out_name}', node=out_name)]
            in_node = _disk_node(in_name,  ix, iy, 'entry', in_lat,  in_lon,  in_edges, rid)
            in_node.add_metadata(**ui_meta_common)
            out_node = _disk_node(out_name, ox, oy, 'exit',  out_lat, out_lon, [], rid)
            out_node.add_metadata(**ui_meta_common)

            new_topo_nodes[in_name] = in_node
            new_topo_nodes[out_name] = out_node
            added.append(rid)
            row_names[rid] = (in_name, out_name)
            # Retain endpoint coords so headland edges can be built by physical
            # end (south/north), not by IN/OUT label — snake ordering flips the
            # label↔end correspondence on alternate rows.
            row_coords[in_name]  = (ix, iy)
            row_coords[out_name] = (ox, oy)

        if not added:
            self.f2c_save_status = 'ERROR: nothing added (all names already taken)'
            return

        # ── Headland edges (point-to-point nav_to_pose) ──────────────────────
        # Connect each node only to its immediate same-end neighbour along the
        # headland, so a route between rows hugs the headland and never angles
        # across a crop row. IN/OUT label is NOT the end: the snake flip puts
        # odd-row IN and even-row OUT at one end, and the opposite labels at the
        # other. Classify by the cross-row coordinate, then chain neighbours
        # sorted along the headland.
        #
        # Cross-row axis = the field dimension with the larger endpoint spread
        # (rows are long and narrow, so the ends are separated along the SHORT
        # axis). We split nodes into two ends by that coordinate's median and,
        # within each end, sort along the other (along-headland) axis and link
        # consecutive nodes. IN→OUT row-follow edges are untouched.
        def _add_headland_edge(p: str, q: str) -> None:
            """Bidirectional nav_to_pose edge p<->q in both graph structures."""

            edge_name = f'{p}_{q}'

            for a, b in ((p, q), (q, p)):
                a_node = new_topo_nodes[a]
                a_node.add_edge(TopoEdge(action=_NAV_ACTION, edge_id=edge_name, node=b))

            for node in new_topo_nodes.values():
                if node.name not in (p, q):
                    continue

                other = q if node.name == p else p
                node.add_edge(other, action=_NAV_ACTION)

        all_pts = dict(row_coords)   # {name: (x, y)}
        for a_name, b_name in _headland_neighbour_pairs(all_pts):
            _add_headland_edge(a_name, b_name)

        if connect_to:
            first_in = row_names[added[0]][0]
            last_out = row_names[added[-1]][1]

            for tgt in (first_in, last_out):
                if tgt not in new_topo_nodes:
                    continue
                node = new_topo_nodes[tgt]
                node.add_edge(connect_to, action=_NAV_ACTION)

        skip_str   = f' (skipped {len(skipped)} dup ids)' if skipped else ''
        splice_str = f' · spliced @ {connect_to}' if connect_to else ' · standalone'
        self.f2c_save_status = (
            f'writing {len(added)} rows · {prefix}{splice_str}{skip_str}…')

        def _modify(file_doc):
            if overwrite:
                old_names = {
                    e.name
                    for e in file_doc.nodes
                    if e.name.startswith(f'{prefix}_R')
                }
                if old_names:
                    file_doc.nodes = [
                        e for e in file_doc.nodes
                        if e.name not in old_names
                    ]
                    # Prune dangling edges pointing at removed nodes
                    for entry in file_doc.nodes:
                        entry.edges = [
                            e for e in entry.edges
                            if e.node not in old_names
                        ]
            existing = {e.name for e in file_doc.nodes}
            for entry in new_topo_nodes.values():
                if entry.name in existing:
                    continue
                file_doc.nodes.append(entry)

        self._persist_and_reload(
            _modify, 'f2c_save_status',
            f'saved {len(added)} rows · {prefix}{splice_str}{skip_str}',
        )

    # ── Repair row connectivity ──────────────────────────────────────────────

    def repair_row_connectivity(self, connect_to: str | None = None) -> None:
        if not self._topo_doc:
            self.f2c_save_status = 'ERROR: map not loaded'
            return

        rows: dict = {}
        coords: dict = {}   # node_name -> (x, y) for same-end classification
        for node in self._topo_doc.nodes:
            meta = node.meta
            rid = meta.get('row_id')
            role = meta.get('row_role')
            if rid is None or role not in ('entry', 'exit'):
                continue
            try:
                rid_int = int(rid)
            except (TypeError, ValueError):
                continue
            rows.setdefault(rid_int, {})[role] = node.name
            # x/y live at the top level of the lightweight node dict
            coords[node.name] = (node.x, node.y)

        if not rows:
            self.f2c_save_status = 'ERROR: no row nodes found'
            return

        sorted_rids = sorted(rows)
        if connect_to and not self._topo_doc.has_node(connect_to):
            self.get_logger().warn(
                f'repair: connect_to={connect_to!r} not in map, ignoring')
            connect_to = None

        wanted_edges: list = []

        # In-row edges: every row's entry -> exit is a row-follow edge. The
        # repair restores these in case they were lost; it must NOT use
        # nav_to_pose here or the robot would drive straight through the crop.
        for rid in sorted_rids:
            inn = rows[rid].get('entry')
            outn = rows[rid].get('exit')
            if inn and outn and inn != outn:
                wanted_edges.append((inn, outn, _ROW_ACTION))
                wanted_edges.append((outn, inn, _ROW_ACTION))

        # Headland edges: same-end neighbours only, classified by geometry —
        # NOT by entry/exit label (snake ordering flips label vs physical end).
        # Shared with the build path so the two cannot diverge.
        if len(coords) >= 2:
            for a_name, b_name in _headland_neighbour_pairs(coords):
                wanted_edges.append((a_name, b_name, _NAV_ACTION))
                wanted_edges.append((b_name, a_name, _NAV_ACTION))
        else:
            self.get_logger().warn(
                'repair: row nodes lack x/y coords — cannot classify headland '
                'ends; skipping headland edges (in-row edges still restored)')

        if connect_to:
            first_in = rows[sorted_rids[0]].get('entry')
            last_out = rows[sorted_rids[-1]].get('exit')
            for tgt in (first_in, last_out):
                if not tgt or tgt == connect_to:
                    continue
                wanted_edges.append((connect_to, tgt, _NAV_ACTION))
                wanted_edges.append((tgt, connect_to, _NAV_ACTION))

        new_topo_nodes = {node.name: node for node in self._topo_doc.nodes}
        added_count = 0
        for src, tgt, _action in wanted_edges:
            if src not in new_topo_nodes or src == tgt:
                continue
            cur = new_topo_nodes[src].edges
            if tgt in cur:
                continue
            cur.append(tgt)
            added_count += 1

        if added_count == 0:
            self.f2c_save_status = (
                f'repair: already wired ({len(sorted_rids)} rows)')
            return

        self.f2c_save_status = f'repair: adding {added_count} edges…'

        def _modify(file_doc):
            for src, tgt, action in wanted_edges:
                if src == tgt:
                    continue
                for entry in file_doc.nodes:
                    if entry.name != src:
                        continue
                    if any(e.name == tgt for e in entry.edges):
                        break
                    entry.add_edge(tgt, action=action)
                    break

        target_str = (f' @ {connect_to}' if connect_to
                      else ' — NO SPLICE, chain still isolated')
        self._persist_and_reload(
            _modify, 'f2c_save_status',
            f'repair: wired {added_count} edges{target_str}',
        )

    # ── Delete topo nodes / rows ─────────────────────────────────────────────

    def delete_topo_node(self, name: str) -> None:
        if not self._topo_doc:
            self.delete_status = 'ERROR: map not loaded'
            return
        if not name or not self._topo_doc.has_node(name):
            self.delete_status = f'ERROR: {name!r} not in map'
            return

        if self.topo_selected == name:
            self.topo_selected = None
        self.delete_status = f'deleting {name}…'

        def _modify(file_doc):
            file_doc.remove_node(name)

        self._persist_and_reload(_modify, 'delete_status', f'deleted {name}')

    def delete_row(self, row_id: int) -> None:
        targets = {node.name for node in self._topo_doc.nodes
                   if node.meta.get('row_id') == row_id}
        if not targets:
            self.delete_status = f'ERROR: no nodes for row {row_id}'
            return
        if not self._topo_doc:
            self.delete_status = 'ERROR: map not loaded'
            return

        if self.topo_selected in targets:
            self.topo_selected = None
        self.delete_status = f'deleting row {row_id} ({len(targets)} nodes)…'

        def _modify(file_doc):
            file_doc.remove_nodes(targets)

        self._persist_and_reload(_modify, 'delete_status',
                                  f'deleted row {row_id} ({len(targets)} nodes)')

    # ── Confirmation dialogs ─────────────────────────────────────────────────

    async def confirm_delete_node(self, name: str | None) -> None:
        if not name or not self._topo_doc.has_node(name):
            return
        nd = self._topo_doc.get_node(name)
        rid = nd.meta.get('row_id')
        with ui.dialog() as d, ui.card():
            ui.label(f'Delete topo node "{name}"?').classes('font-semibold')
            if rid is not None:
                ui.label(
                    f'This is part of row {rid}. To delete the whole row '
                    f'(entry + exit), use the ✕ on the Mission tab instead.'
                ).classes('text-xs').style('color:#9a6700;max-width:340px')
            ui.label('This persists immediately and cannot be undone.').classes(
                'text-xs').style('color:#8c959f')
            with ui.row().classes('w-full justify-end gap-2 mt-2'):
                ui.button('Cancel', on_click=lambda: d.submit('cancel')).props('flat no-caps')
                ui.button('Delete', color='negative',
                          on_click=lambda: d.submit('ok')).props('no-caps')
        if await d == 'ok':
            self.delete_topo_node(name)

    async def confirm_delete_row(self, row_id: int) -> None:
        targets = sorted(node.name for node in self._topo_doc.nodes
                         if node.meta.get('row_id') == row_id)
        if not targets:
            return
        with ui.dialog() as d, ui.card():
            ui.label(f'Delete row {row_id}?').classes('font-semibold')
            ui.label(f'{len(targets)} nodes will be removed:').classes('text-xs').style('color:#57606a')
            ui.label(', '.join(targets)).classes('text-xs font-mono').style(
                'color:#8c959f;max-width:340px;word-break:break-all')
            with ui.row().classes('w-full justify-end gap-2 mt-2'):
                ui.button('Cancel', on_click=lambda: d.submit('cancel')).props('flat no-caps')
                ui.button('Delete', color='negative',
                          on_click=lambda: d.submit('ok')).props('no-caps')
        if await d == 'ok':
            self.delete_row(row_id)

    # ── existing helpers below ───────────────────────────────────────────────

    def _patch_node_role(self, node_name: str, role: str) -> None:
        map_name = self._topo_doc.name
        map_file = f'/workspace/maps/{map_name}'
        def _write():
            try:
                if not os.path.exists(map_file):
                    return
                doc = parse_topo_yaml(map_file)
                if doc.has_node(node_name):
                    node = doc.get_node(node_name)
                    node.patch_role(role)
                dump_topo_yaml(doc, map_file)
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
            with ui.tab_panel(tab_nav):
                self._nav_content()
            with ui.tab_panel(tab_mission):
                self._mission_content()
            with ui.tab_panel(tab_system):
                self._system_content()

    # ── Nav tab ───────────────────────────────────────────────────────────────

    def _nav_content(self) -> None:

        with ui.row().classes('w-full gap-3 items-stretch'):

            with ui.column().classes('flex-1 gap-3').style('min-width:0'):

                with ui.row().classes('w-full gap-3 items-stretch'):

                    with ui.card().style('padding:16px 20px;flex-shrink:0'):
                        ui.html('<div class="sec-label mb-3">Joystick</div>')
                        with ui.row().classes('items-center gap-6'):
                            def _debug_joystick_move(e):
                                self.get_logger().info(
                                    f'[joystick debug] raw e.x={e.x!r} e.y={e.y!r} '
                                    f'(as float: x={float(e.x):+.3f} y={float(e.y):+.3f})')
                                self.send_speed(float(e.y), float(e.x))

                            ui.joystick(
                                color='#1a7f37', size=130,
                                on_move=_debug_joystick_move,
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

                    with ui.card().classes('flex-1').style('padding:12px;min-width:0'):
                        ui.html('<div class="sec-label mb-2">Node Map</div>')
                        # Node map + robot marker overlay (same viewBox, so they
                        # align). Overlay is pointer-events:none so clicks reach
                        # the nodes; it updates on movement without rebuilding
                        # the clickable node DOM.
                        with ui.element('div').classes('relative w-full'):
                            map_html = ui.html(
                                _build_svg(self._topo_doc, None, None)
                            ).classes('w-full')
                            robot_html = ui.html(
                                _build_robot_svg(self._topo_doc.nodes, None)
                            ).classes('absolute top-0 left-0 w-full').style(
                                'pointer-events:none')

                with ui.row().classes('w-full gap-3 items-start'):

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

                    with ui.card().classes('flex-1').style('padding:12px 14px'):
                        ui.label('Row Discovery').classes('font-semibold mb-2')
                        ui.html(
                            '<div style="font-size:12px;color:#9a6700;'
                            'background:#fff8dc;border:1px solid #d4a72c;'
                            'border-radius:4px;padding:6px 8px;margin-bottom:8px;">'
                            '⚠ Discovery mode drives the robot autonomously '
                            'through the field with no operator override. '
                            'Only enable this with the person-detection '
                            'safety system active and confirmed running.</div>'
                        )
                        with ui.row().classes('items-center gap-2'):
                            discovery_checkbox = ui.checkbox('Discovery mode').bind_value(
                                self, 'discovery_active')
                            ui.label().classes('text-xs font-mono ml-2').bind_text_from(
                                self, 'discovery_status')

                        async def _on_discovery_change(e, _cb=discovery_checkbox) -> None:
                            if e.args:  # ticked on
                                with ui.dialog() as d, ui.card():
                                    ui.label(
                                        'Confirm person-detection safety system'
                                    ).classes('font-semibold')
                                    ui.label(
                                        'Discovery mode will drive the robot with '
                                        'no operator override. Confirm the '
                                        'person-detection safety system is running '
                                        'and active before continuing.'
                                    ).classes('text-sm').style('max-width:320px')
                                    with ui.row().classes('justify-end gap-2 mt-2'):
                                        ui.button('Cancel',
                                            on_click=lambda: d.submit('cancel')).props(
                                            'flat no-caps')
                                        ui.button('Confirmed — Start', color='positive',
                                            on_click=lambda: d.submit('go')).props(
                                            'no-caps')
                                result = await d
                                if result != 'go':
                                    _cb.value = False
                                    return
                                self.start_discovery()
                            else:
                                self.stop_discovery()

                        discovery_checkbox.on('update:model-value', _on_discovery_change)

                    # OBSTACLE: Mark Obstacle card next to Drop Node
                    attach_nav_card(self, self._obstacle_mgr)

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
                del_btn  = ui.button('Delete Node', color='negative').classes('w-full').props(
                    'no-caps outline')

                ui.html('<div class="sec-label mt-3">Status</div>')
                del_status_lbl = ui.label('').classes('text-xs font-mono').style(
                    'color:#57606a;word-break:break-all')

                ui.html('<div class="sec-label mt-3">Nodes</div>')
                with ui.scroll_area().style('flex:1;min-height:0'):
                    node_col = ui.column().style('gap:1px;width:100%')

        go_btn.on_click(
            lambda: self.send_nav_goal(self.topo_selected) if self.topo_selected else None)
        stop_btn.on_click(self.cancel_nav_goal)
        del_btn.on_click(lambda: self.confirm_delete_node(self.topo_selected))

        def on_node_clicked(e) -> None:
            n = (e.args or {}).get('node')
            if n and self._topo_doc.has_node(n):
                self.topo_selected = n
        ui.on('topo_node_clicked', on_node_clicked)

        def inject_click_js() -> None:
            # Event delegation: a single document-level listener (attached once,
            # guarded) that resolves clicks to the nearest .topo-node. Survives
            # map redraws — unlike per-element onclick, which gets wiped every
            # time the SVG is rebuilt (on current-node change), leaving the
            # nodes unclickable.
            ui.run_javascript("""
                if (!window.__topoDelegated) {
                    window.__topoDelegated = true;
                    document.addEventListener('click', (e) => {
                        const el = e.target.closest('.topo-node');
                        if (el) emitEvent('topo_node_clicked', {node: el.dataset.node});
                    });
                }
            """)

        _prev: dict = {}

        def refresh_nav() -> None:
            odom = self.latest_odom
            gps  = self.latest_gps
            if odom is not None:
                px, py  = odom.pose.pose.position.x, odom.pose.pose.position.y
                gps_str = (f'\n{gps.latitude:.5f}\n{gps.longitude:.5f}'
                        if gps and gps.status.status >= 0 else '')
                pose_lbl.set_text(f'({px:.2f}, {py:.2f}){gps_str}')
            else:
                pose_lbl.set_text('no odom')

            cur = self.topo_current
            if cur and cur != '—':
                cur_drop_lbl.set_text(f'→ {cur}')
                cur_drop_lbl.style('color:#1a7f37')
            else:
                cur_drop_lbl.set_text('no current node')
                cur_drop_lbl.style('color:#8c959f')
            row_hint.set_text(_ROW_ACTION if row_id_input.value else _NAV_ACTION)
            row_hint.style('color:#0969da' if row_id_input.value else 'color:#8c959f')
            drop_status_lbl.set_text(self.drop_status)
            drop_status_lbl.style(
                'color:#cf222e' if self.drop_status.startswith('ERROR') else 'color:#1a7f37')

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

            rp = self._robot_pose()
            rp_key = None if rp is None else (round(rp[0], 1), round(rp[1], 1),
                                            round(rp[2], 2))
            snap = {'sel': self.topo_selected, 'cur': self.topo_current,
                    'stat': self.topo_nav_status, 'nav': self.topo_navigating,
                    'nodes': set(self._topo_doc.nodes), 'robot': rp_key}
            nonlocal _prev
            changed = {k for k, v in snap.items() if _prev.get(k) != v}
            if not changed:
                return
            _prev.update(snap)

            if changed & {'robot', 'nodes'}:
                robot_html.set_content(_build_robot_svg(self._topo_doc.nodes, rp))

            if changed & {'sel', 'cur', 'nodes'}:
                map_html.set_content(
                    _build_svg(self._topo_doc, self.topo_selected,
                            self.topo_current))
                inject_click_js()

            if changed & {'sel', 'nodes'}:
                node_col.clear()
                with node_col:
                    node_names = sorted(self._topo_doc.nodes, key=attrgetter('name'))
                    for nd in node_names:
                        is_row   = nd.meta.get('row_id') is not None
                        rid_v    = nd.meta.get('row_id', '')
                        rrole_v  = nd.meta.get('row_role', '')
                        glat     = nd.meta.get('gps_lat', '')
                        glon     = nd.meta.get('gps_lon', '')
                        title    = (f'Row {rid_v} {rrole_v} | {glat} {glon}'.strip()
                                    if is_row else f'{glat} {glon}'.strip())
                        cls      = ('node-item'
                                    + (' sel' if nd.name == self.topo_selected else '')
                                    + (' row' if is_row else ''))
                        n = nd.name
                        ui.html(
                            f'<div class="{cls}" title="{title}">'
                            f'{nd.name}{"  · row" if is_row else ""}</div>'
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
            del_btn.set_enabled(bool(self.topo_selected))
            del_status_lbl.set_text(self.delete_status)
            del_status_lbl.style(
                'color:#cf222e' if self.delete_status.startswith('ERROR') else
                'color:#1a7f37' if self.delete_status else 'color:#57606a')

        ui.timer(0.2, refresh_nav)
        inject_click_js()

    # ── Mission tab ───────────────────────────────────────────────────────────

    def _mission_content(self) -> None:
        corners_ll: list[tuple[float, float]] = []
        swath_layers: list = []
        poly_layer:   list = [None]

        with ui.row().classes('w-full gap-3 items-start mb-3'):

            with ui.card().classes('flex-1').style('padding:10px;min-width:0'):
                ui.html('<div class="sec-label mb-2">Field boundary — click to draw</div>')
                gps_center = (
                    (self.latest_gps.latitude, self.latest_gps.longitude)
                    if self.latest_gps else (9.045094, 77.792024)
                )
                mission_map = ui.leaflet(center=gps_center, zoom=18).classes('w-full h-96')
                mission_map.tile_layer(
                    url_template='https://server.arcgisonline.com/ArcGIS/rest/services/'
                                 'World_Imagery/MapServer/tile/{z}/{y}/{x}',
                    options={'attribution': 'Esri', 'maxZoom': 20},
                )

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

                # HEADLAND: shrink cover area by this much on all sides so
                # swaths don't start/end at the field boundary. 0 = off.
                ui.html('<div class="sec-label mt-3">Headland width</div>')
                f2c_headland = ui.number(
                    value=0.0, min=0.0, max=5.0, step=0.1, precision=2,
                    suffix='m',
                ).classes('w-full')
                ui.label('0 = no inset; ≈ tool width for one-row headland').classes(
                    'text-xs').style('color:#8c959f')

                # SNAKE: reverse every other swath so end-of-N is near
                # start-of-N+1. Default on.
                ui.html('<div class="sec-label mt-3">Snake order</div>')
                f2c_snake = ui.checkbox('reverse every other row', value=True)

                ui.html('<div class="sec-label mt-3">First row ID</div>')
                f2c_row_id_start = ui.number(
                    value=1, min=1, step=1, precision=0,
                ).classes('w-full')

                ui.html('<div class="sec-label mt-3">Row name prefix</div>')
                f2c_prefix = ui.input(
                    value='F2C', placeholder='F2C',
                ).classes('w-full')
                ui.label('→ {prefix}_R{n}_IN / _OUT').classes('text-xs font-mono').style(
                    'color:#8c959f')

                # OBSTACLE: draw-mode + shape + radius + padding + map-click
                # dispatch. Boundary-click delegates here to the local F2C
                # corner-drawing closure.
                def _boundary_click(lat: float, lon: float) -> None:
                    corners_ll.append((lat, lon))
                    mission_map.marker(latlng=(lat, lon))
                    _redraw_polygon()

                draw_handle = attach_mission_sidebar_controls(
                    self, self._obstacle_mgr, mission_map, _boundary_click)
                obstacle_pad = draw_handle.obstacle_pad

                ui.separator().classes('my-3')

                corners_lbl = ui.label('0 corners').classes('text-xs font-mono').style(
                    'color:#57606a')

                plan_btn  = ui.button('Plan Rows').props(
                    'color=positive no-caps').classes('w-full mt-2')
                save_btn  = ui.button('Save as Topo Rows').props(
                    'color=primary no-caps').classes('w-full mt-1')
                save_btn.set_enabled(False)
                f2c_overwrite = ui.checkbox('Overwrite existing rows with same prefix',
                                            value=False).classes('text-xs mt-1')
                clear_btn = ui.button('Clear').props(
                    'outline no-caps').classes('w-full mt-1')

                repair_btn = ui.button('Repair Connectivity').props(
                    'outline no-caps').classes('w-full mt-1')
                repair_btn.tooltip(
                    'Wire navigate_to_pose edges between consecutive row '
                    'IN/OUT pairs. Splices into current node if localised.')

                f2c_status = ui.label('').classes('text-xs font-mono mt-2').style(
                    'color:#57606a;word-break:break-word')
                f2c_save_lbl = ui.label('').classes('text-xs font-mono mt-1').style(
                    'color:#57606a;word-break:break-word')

        # OBSTACLE: map-click is wired by attach_mission_sidebar_controls.
        # It dispatches to _boundary_click when in boundary mode, to the
        # manager otherwise. Do NOT add a second mission_map.on('map-click')
        # handler here — it would fire alongside ours and double-draw.

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
                    name='polygon',
                    args=[latlngs,
                          {'color': '#1a7f37', 'fillOpacity': 0.15,
                           'weight': 2, 'dashArray': '6 4'}],
                )
            corners_lbl.set_text(
                f'{len(corners_ll)} corner{"s" if len(corners_ll) != 1 else ""}' +
                (' ✓' if len(corners_ll) >= 3 else ' — need 3+'))

        def do_clear():
            corners_ll.clear()
            for lyr in swath_layers:
                try:
                    lyr.run_method('remove')
                except Exception:
                    pass
            swath_layers.clear()
            if poly_layer[0] is not None:
                try:
                    poly_layer[0].run_method('remove')
                except Exception:
                    pass
                poly_layer[0] = None
            # OBSTACLE: also tear down any in-progress obstacle polygon
            draw_handle.clear_in_progress()
            corners_lbl.set_text('0 corners')
            f2c_status.set_text('')
            save_btn.set_enabled(False)
            mission_map.set_center(mission_map.center)

        clear_btn.on_click(do_clear)

        async def do_plan():
            if len(corners_ll) < 3:
                f2c_status.set_text('Need at least 3 corners')
                f2c_status.style('color:#cf222e')
                return

            plan_btn.set_enabled(False)
            f2c_status.set_text('Running F2C…')
            f2c_status.style('color:#57606a')

            width      = float(f2c_width.value or 1.2)
            angle_deg  = float(f2c_angle.value or 0)
            row_start  = int(f2c_row_id_start.value or 1)
            headland_m = float(f2c_headland.value or 0.0)
            snake      = bool(f2c_snake.value)

            # OBSTACLE: snapshot obstacle rings and pad
            obstacle_rings = self._obstacle_mgr.rings_ll()
            pad_m          = float(obstacle_pad.value or 0.0)

            try:
                swaths = await ng_run.io_bound(
                    _run_f2c, list(corners_ll), obstacle_rings,
                    width, angle_deg, pad_m,
                    headland_m, snake)
            except Exception as exc:
                f2c_status.set_text(f'ERROR: {exc}')
                f2c_status.style('color:#cf222e')
                plan_btn.set_enabled(True)
                return

            for lyr in swath_layers:
                try:
                    lyr.run_method('remove')
                except Exception:
                    pass
            swath_layers.clear()

            for pts in swaths:
                latlngs = [[lat, lon] for lat, lon in pts]
                lyr = mission_map.generic_layer(
                    name='polyline',
                    args=[latlngs,
                          {'color': '#0969da', 'weight': 2, 'opacity': 0.85}],
                )
                swath_layers.append(lyr)

            self._f2c_swaths     = swaths
            self._f2c_row_start  = row_start
            self._f2c_tool_width = width
            self._f2c_angle_deg  = angle_deg
            # Field reference origin = first boundary corner, the same lat0/lon0
            # _run_f2c projected from. The save path re-anchors to this so the
            # lat/lon round-trip cancels and rows land at the local odom origin
            # — instead of being offset by the distance between the field and
            # whatever latest_gps happened to read (in sim, the datum fix).
            self._f2c_origin_ll = tuple(corners_ll[0]) if corners_ll else None

            hl_note  = f' · {headland_m}m headland' if headland_m > 0 else ''
            snk_note = ' · snake' if snake else ''
            obs_note = (f' · {len(obstacle_rings)} obs avoided'
                        if obstacle_rings else '')
            f2c_status.set_text(
                f'{len(swaths)} rows · {width}m wide · {angle_deg:.0f}°'
                f'{hl_note}{snk_note}{obs_note}')
            f2c_status.style('color:#1a7f37')
            plan_btn.set_enabled(True)
            save_btn.set_enabled(bool(swaths))

        plan_btn.on_click(do_plan)

        def do_save():
            self.save_f2c_rows_to_topo(
                f2c_prefix.value or 'F2C',
                int(f2c_row_id_start.value or 1),
                overwrite=f2c_overwrite.value,
            )
        save_btn.on_click(do_save)

        async def do_repair():
            cur = self.topo_current
            default_base = ''
            if cur not in ('—', 'none', 'None', '', None) and self._topo_doc.has_node(cur):
                default_base = cur
            elif self.topo_selected and self._topo_doc.has_node(self.topo_selected):
                default_base = self.topo_selected

            row_count = sum(
                1 for nd in self._topo_doc.nodes
                if nd.meta.get('row_id') is not None
                and nd.meta.get('row_role') == 'entry'
            )

            with ui.dialog() as d, ui.card():
                ui.label('Repair row connectivity').classes('font-semibold')
                ui.label(
                    f'Add navigate_to_pose edges between {row_count} consecutive '
                    f'row IN/OUT pairs. Optionally splice the chain into a base '
                    f'node so the planner can reach it.'
                ).classes('text-xs').style('color:#57606a;max-width:340px')
                base_input = ui.input(
                    label='Splice into',
                    placeholder='leave blank for chain only',
                    value=default_base,
                ).classes('w-full mt-2')
                ui.label(
                    'Without a base node the chain is wired internally but '
                    'remains unreachable from the rest of the graph.'
                ).classes('text-xs').style(
                    'color:#9a6700;max-width:340px;margin-top:4px')
                with ui.row().classes('w-full justify-end gap-2 mt-2'):
                    ui.button('Cancel',
                              on_click=lambda: d.submit('cancel')).props(
                                  'flat no-caps')
                    ui.button('Repair', color='positive',
                              on_click=lambda: d.submit('ok')).props('no-caps')

            result = await d
            if result != 'ok':
                return

            base = (base_input.value or '').strip()
            if base and not self._topo_doc.has_node(base):
                self.f2c_save_status = f'ERROR: base node {base!r} not in map'
                return
            self.repair_row_connectivity(connect_to=base or None)

        repair_btn.on_click(do_repair)

        _save_prev = ['']
        def _refresh_save_status():
            cur = self.f2c_save_status
            if cur == _save_prev[0]:
                return
            _save_prev[0] = cur
            f2c_save_lbl.set_text(cur)
            f2c_save_lbl.style(
                'color:#cf222e' if cur.startswith('ERROR') else
                'color:#1a7f37' if cur else 'color:#57606a')
        ui.timer(0.4, _refresh_save_status)

        # ─────────────────────────────────────────────────────────────────────
        # MISSION QUEUE CARD
        # ─────────────────────────────────────────────────────────────────────

        with ui.card().classes('w-full'):
            with ui.row().classes('items-baseline gap-2 mb-3'):
                ui.label('Mission Queue').classes('font-semibold')
                ui.label('select rows · pick action · run').classes(
                    'text-xs').style('color:#8c959f')

            # ── action selector + param editor ────────────────────────────────
            with ui.row().classes('items-center gap-3 w-full mb-2 flex-wrap'):
                ui.html('<div class="sec-label" style="white-space:nowrap">Implement</div>')
                action_select = ui.select(
                    options={k: f'{a.icon} {a.label}' for k, a in ACTIONS.items()},
                    value='drive',
                ).classes('flex-1').props('dense outlined')

            # Param inputs rendered dynamically when action changes.
            param_row = ui.row().classes('items-end gap-3 w-full flex-wrap mb-2')
            param_inputs: dict = {}   # key → ui.number widget

            def _rebuild_params():
                param_row.clear()
                param_inputs.clear()
                action = ACTIONS.get(action_select.value)
                if action is None or not action.param_schema:
                    return
                with param_row:
                    for p in action.param_schema:
                        inp = ui.number(
                            label=f'{p.label} ({p.unit})',
                            value=p.default,
                            min=p.min, max=p.max, step=p.step,
                            precision=p.precision,
                        ).classes('w-32').props('dense outlined')
                        param_inputs[p.key] = inp

            action_select.on_value_change(lambda _: _rebuild_params())
            _rebuild_params()

            # ── available / queue panels ──────────────────────────────────────
            with ui.row().classes('w-full gap-4 items-start'):
                with ui.card().classes('flex-1').style('background:#f6f8fa;padding:10px'):
                    ui.html('<div class="sec-label mb-2">Available rows</div>')
                    available_col = ui.column().style('gap:2px;width:100%')
                with ui.card().classes('flex-1').style('background:#f6f8fa;padding:10px'):
                    ui.html('<div class="sec-label mb-2">Today\'s queue</div>')
                    queue_col = ui.column().style('gap:2px;width:100%')
                    queue_lbl = ui.label('Empty — add rows from the left').classes(
                        'text-xs').style('color:#8c959f')

            # mission_queue: list of (row_id, action_key, action_params) triples
            mission_queue: list = []

            def _render_queue():
                queue_col.clear()
                queue_lbl.set_visibility(not mission_queue)
                if not mission_queue:
                    return
                with queue_col:
                    for i, (rid, act, params) in enumerate(mission_queue):
                        idx = i
                        adef = ACTIONS.get(act)
                        param_str = ' · '.join(
                            f'{p.label}: {params.get(p.key, p.default):.{p.precision}f}{p.unit}'
                            for p in (adef.param_schema if adef else [])
                        )
                        with ui.row().classes('items-center gap-1 w-full'):
                            with ui.column().classes('flex-1 gap-0'):
                                ui.label(
                                    f'Row {rid}  {adef.icon if adef else "?"} {adef.label if adef else act}'
                                ).classes('text-sm font-mono')
                                if param_str:
                                    ui.label(param_str).classes('text-xs font-mono').style(
                                        'color:#8c959f')
                            ui.button('↑',
                                on_click=lambda _, i=idx: _move(i, -1)).props(
                                'flat dense').classes('text-xs').style(
                                'color:#57606a').set_enabled(i > 0)
                            ui.button('↓',
                                on_click=lambda _, i=idx: _move(i, 1)).props(
                                'flat dense').classes('text-xs').style(
                                'color:#57606a').set_enabled(i < len(mission_queue) - 1)
                            ui.button('✕',
                                on_click=lambda _, i=idx: _remove(i)).props(
                                'flat dense').classes('text-xs').style('color:#cf222e')

            def _move(idx, d):
                ni = idx + d
                if 0 <= ni < len(mission_queue):
                    mission_queue[idx], mission_queue[ni] = mission_queue[ni], mission_queue[idx]
                _render_queue()

            def _remove(idx):
                mission_queue.pop(idx)
                _render_queue()

            def _add_row(row_id):
                act = action_select.value or 'drive'
                adef = ACTIONS.get(act)
                params = {p.key: float(param_inputs[p.key].value)
                          for p in (adef.param_schema if adef else [])
                          if p.key in param_inputs}
                # allow duplicates — operator may want to run the same row
                # twice with different implements (e.g. spray then harvest)
                mission_queue.append((row_id, act, params))
                _render_queue()

            mission_status = ui.label('').classes('text-xs font-mono mt-3').style(
                'color:#57606a')

            with ui.row().classes('gap-2 mt-3'):
                run_btn = ui.button(
                    'Run Mission',
                    on_click=lambda: self._run_mission(mission_queue, mission_status),
                ).props('color=positive no-caps')
                ui.button(
                    'Cancel',
                    on_click=self.cancel_mission,
                ).props('color=negative no-caps flat')

            # ── available rows refresh ────────────────────────────────────────
            _avail_prev: list[set[TopoNode]] = [set()]
            def _refresh_available():
                nonlocal _avail_prev
                snap = set(self._topo_doc.nodes)
                if snap != _avail_prev[0]:
                    return
                _avail_prev[0] = snap
                rows: dict[int, str] = {}
                for nd in self._topo_doc.nodes:
                    meta = nd.meta
                    rid  = meta.get('row_id')
                    if rid is not None and meta.get('row_role', '') == 'entry':
                        try:
                            rows[int(rid)] = nd.name
                        except (TypeError, ValueError):
                            pass
                available_col.clear()
                if not rows:
                    with available_col:
                        ui.label('No rows in map yet').classes('text-xs').style(
                            'color:#8c959f')
                    return
                with available_col:
                    for rid in sorted(rows):
                        r = rid
                        with ui.row().classes('items-center gap-2 w-full'):
                            ui.label(f'Row {rid}').classes('text-sm font-mono flex-1')
                            ui.label(rows[rid]).classes('text-xs font-mono').style(
                                'color:#8c959f')
                            ui.button(
                                'Add →',
                                on_click=lambda _, r=r: _add_row(r),
                            ).props('color=primary outline no-caps dense')
                            ui.button(
                                '✕',
                                on_click=lambda _, r=r: self.confirm_delete_row(r),
                            ).props('flat dense').classes('text-xs').style('color:#cf222e')

            ui.timer(1.0, _refresh_available)

            # ── mission store panel ───────────────────────────────────────────
            ui.separator().classes('my-3')
            with ui.row().classes('items-baseline gap-2 mb-2'):
                ui.label('Mission Store').classes('font-semibold')
                ui.label('saved missions with repeat schedules').classes(
                    'text-xs').style('color:#8c959f')

            with ui.card().classes('w-full').style('background:#f6f8fa;padding:10px'):
                missions_col = ui.column().style('gap:2px;width:100%')
                missions_empty_lbl = ui.label(
                    'No saved missions'
                ).classes('text-xs').style('color:#8c959f')

            # Save current queue as a named recurring mission
            with ui.row().classes('items-center gap-2 mt-2 flex-wrap'):
                save_name_input = ui.input(
                    placeholder='Mission name', label='Save queue as…',
                ).classes('flex-1').props('dense outlined')
                save_repeat = ui.number(
                label='Repeat (h)', value=None, min=1, step=1, precision=0,
                ).classes('w-24').props('dense outlined clearable').tooltip('Leave blank for one-shot')

                def _save_mission():
                    if not mission_queue:
                        # This is a protected method, we should probably find a better way of bubbling up the error.
                        self._mission_store._set_status('ERROR: queue is empty')  # pylint: disable=protected-access
                        return
                    rows_for_store = [
                        next(
                            (nd.name for nd in self._topo_doc.nodes
                             if nd.meta.get('row_id') == rid
                             and nd.meta.get('row_role') == 'entry'),
                            f'ROW_{rid}_IN',
                        )
                        for rid, _, _ in mission_queue
                    ]
                    # All steps in the queue share the action+params of the
                    # first entry.  Mixed-action missions aren't supported in
                    # the store yet — first step wins.
                    first_act    = mission_queue[0][1]
                    first_params = mission_queue[0][2]
                    rpt = int(save_repeat.value) if save_repeat.value else None
                    self._mission_store.add(
                        rows=rows_for_store,
                        action=first_act,
                        action_params=first_params,
                        name=save_name_input.value or '',
                        repeat_every_hours=rpt,
                        active=True,
                    )

                ui.button('Save', on_click=_save_mission).props(
                    'color=primary no-caps dense')

            mission_store_status = ui.label('').classes('text-xs font-mono mt-1').style(
                'color:#57606a')

            _mstore_prev = [-1]
            def _refresh_missions():
                # Store status line
                cur_status = self.mission_status
                mission_store_status.set_text(cur_status)
                mission_store_status.style(
                    'color:#cf222e' if cur_status.startswith('ERROR') else
                    'color:#1a7f37' if cur_status else 'color:#57606a')

                v = self.missions_version
                if v == _mstore_prev[0]:
                    return
                _mstore_prev[0] = v

                snap = self.missions
                missions_col.clear()
                missions_empty_lbl.set_visibility(not snap)
                run_btn.set_enabled(not self._mission_running)

                if not snap:
                    return
                with missions_col:
                    for m in snap:
                        mid   = m.get('id', '?')
                        name  = m.get('name', mid)
                        act   = m.get('action', '—')
                        adef  = ACTIONS.get(act)
                        icon  = adef.icon if adef else '?'
                        rows  = m.get('rows', [])
                        active = m.get('active', False)
                        due_h = self._mission_store.next_due_in_hours(mid)
                        if due_h is None:
                            due_str, due_col = 'done', 'color:#8c959f'
                        elif due_h == 0.0:
                            due_str, due_col = 'due now', 'color:#1a7f37'
                        else:
                            due_str, due_col = f'in {due_h:.1f}h', 'color:#9a6700'
                        last_ok = m.get('last_run_success')
                        last_str = '✓' if last_ok is True else '✗' if last_ok is False else '—'

                        with ui.row().classes('items-center gap-2 w-full'):
                            ui.label(f'{icon} {name}').classes(
                                'text-sm font-mono').style('min-width:100px')
                            ui.label(f'{len(rows)} rows').classes(
                                'text-xs font-mono flex-1').style('color:#8c959f')
                            ui.label(due_str).classes('text-xs font-mono').style(due_col)
                            ui.label(last_str).classes('text-xs font-mono').style(
                                'color:#1a7f37' if last_ok is True else
                                'color:#cf222e' if last_ok is False else 'color:#8c959f')
                            act_toggle = ui.checkbox('', value=active).props('dense')
                            act_toggle.tooltip('Active — included in today_queue()')
                            act_toggle.on_value_change(
                                lambda e, m=mid: self._mission_store.set_active(m, e.value))
                            ui.button(
                                '✕',
                                on_click=lambda _, m=mid: self._mission_store.delete(m),
                            ).props('flat dense').classes('text-xs').style('color:#cf222e')

            ui.timer(0.5, _refresh_missions)

        # OBSTACLE: obstacle list + map rendering, full width below the queue.
        attach_mission_obstacle_panel(draw_handle)

# ─────────────────────────────────────────────────────────────────────────────
# NiceGuiNode mission executor methods
# ─────────────────────────────────────────────────────────────────────────────

    def _get_tool_publisher(self, topic: str, is_float: bool = False):
        """Return (creating if needed) a publisher for the given tool topic.
        is_float=True → std_msgs/Float64; False → std_msgs/Bool."""
        key = (topic, is_float)
        if key not in self._tool_publishers:
            if is_float:
                self._tool_publishers[key] = self.create_publisher(Float64, topic, 1)
            else:
                self._tool_publishers[key] = self.create_publisher(Bool, topic, 1)
        return self._tool_publishers[key]

    def _publish_tool_msgs(self, action_key: str,
                           action_params: dict | None,
                           enable: bool) -> None:
        """Publish all (topic, value) pairs from action_ros_msgs."""
        for topic, value in action_ros_msgs(action_key, action_params, enable):
            try:
                if isinstance(value, bool):
                    pub = self._get_tool_publisher(topic, is_float=False)
                    msg = Bool()
                    msg.data = value
                else:
                    pub = self._get_tool_publisher(topic, is_float=True)
                    msg = Float64()
                    msg.data = float(value)
                pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(
                    f'_publish_tool_msgs({topic}, enable={enable}): {exc}')

    def _run_mission(self, queue: list, status_lbl) -> None:
        """Launch a mission from the UI queue.

        queue: list of (row_id, action_key, action_params) triples.
        Each queued row is TWO nav goals, not one:
          1. transit to the row's entry node (implement OFF — this may be
             headland travel through other rows/edges)
          2. entry -> exit (implement ON) — this is the leg
             topological_navigation actually resolves to a limbic_row_follow
             edge, i.e. the row is actually traversed here.
        Sending only the entry-node goal (previous behaviour) meant a
        queued row was never actually driven through at all — any row
        that DID get row-followed was only a side effect of the router's
        path to reach a LATER row's entry passing through this row's exit.
        A single-row mission, or the last row in a multi-row queue, was
        silently never traversed.
        """
        if not queue:
            status_lbl.set_text('ERROR: queue is empty')
            status_lbl.style('color:#cf222e')
            return
        if self._mission_running:
            status_lbl.set_text('ERROR: mission already running')
            status_lbl.style('color:#cf222e')
            return
        if not _ACTION_OK:
            status_lbl.set_text('ERROR: action client unavailable')
            status_lbl.style('color:#cf222e')
            return

        # Resolve row_id → (entry_node, exit_node) from current topo map.
        row_entry: dict[int, str] = {}
        row_exit:  dict[int, str] = {}
        for nd in self._topo_doc.nodes:
            meta = nd.meta
            rid  = meta.get('row_id')
            role = meta.get('row_role', '')
            if rid is None:
                continue
            try:
                rid_int = int(rid)
            except (TypeError, ValueError):
                continue
            if role == 'entry':
                row_entry[rid_int] = nd.name
            elif role == 'exit':
                row_exit[rid_int] = nd.name

        missing_entry = [rid for rid, _, _ in queue if rid not in row_entry]
        missing_exit  = [rid for rid, _, _ in queue if rid not in row_exit]
        if missing_entry or missing_exit:
            missing = sorted(set(missing_entry) | set(missing_exit))
            status_lbl.set_text(f'ERROR: incomplete row nodes for row(s) {missing}')
            status_lbl.style('color:#cf222e')
            return

        steps = [(rid, row_entry[rid], row_exit[rid], act, params)
                 for rid, act, params in queue]

        self._mission_running = True
        self._mission_cancel  = False
        status_lbl.set_text(f'Starting {len(steps)} step(s)…')
        status_lbl.style('color:#57606a')

        def _execute():
            success_overall = True
            for step_idx, (rid, entry_node, exit_node, action, params) in enumerate(steps):
                if self._mission_cancel or self.soft_estop_active:
                    status_lbl.set_text('Cancelled')
                    status_lbl.style('color:#9a6700')
                    success_overall = False
                    break

                adef = ACTIONS.get(action)
                label = f'{adef.icon} {adef.label}' if adef else action

                # Leg 1: transit to entry — implement OFF, this isn't the row yet.
                status_lbl.set_text(
                    f'[{step_idx+1}/{len(steps)}] Row {rid} → transit to {entry_node}')
                status_lbl.style('color:#0969da')
                nav_ok = self._send_goal_sync(entry_node)
                if not nav_ok:
                    status_lbl.set_text(
                        f'Row {rid}: transit to {entry_node} failed — stopping mission')
                    status_lbl.style('color:#cf222e')
                    success_overall = False
                    break

                if self._mission_cancel or self.soft_estop_active:
                    status_lbl.set_text('Cancelled')
                    status_lbl.style('color:#9a6700')
                    success_overall = False
                    break

                # Leg 2: entry -> exit — implement ON, this is the row itself.
                status_lbl.set_text(
                    f'[{step_idx+1}/{len(steps)}] Row {rid} {label} → {exit_node}')
                status_lbl.style('color:#0969da')

                self._publish_tool_msgs(action, params, enable=True)
                nav_ok = self._send_goal_sync(exit_node)
                self._publish_tool_msgs(action, params, enable=False)

                if not nav_ok:
                    status_lbl.set_text(
                        f'Row {rid}: traversal to {exit_node} failed — stopping mission')
                    status_lbl.style('color:#cf222e')
                    success_overall = False
                    break

            if success_overall:
                status_lbl.set_text(
                    f'Mission complete — {len(steps)} step(s) done ✓')
                status_lbl.style('color:#1a7f37')

            self._mission_running = False
            self._mission_cancel  = False

        threading.Thread(target=_execute, daemon=True).start()

    def _send_goal_sync(self, target: str, timeout_sec: float = 300.0) -> bool:
        """Send a GotoNode goal and block until success/failure/cancel/timeout.
        Returns True on success.  Runs in the executor thread."""
        if not _ACTION_OK:
            return False

        done_event = threading.Event()
        result_holder: list = [None]

        if not self._nav_ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('_send_goal_sync: action server not ready')
            return False

        goal = GotoNode.Goal()
        goal.target = target
        self.topo_nav_status = f'→ {target}'
        self.topo_navigating = True

        def _on_accepted(future):
            gh = future.result()
            if not gh.accepted:
                result_holder[0] = False
                self.topo_nav_status = 'goal rejected'
                self.topo_navigating = False
                done_event.set()
                return
            self._nav_goal_handle = gh
            gh.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            success = getattr(future.result().result, 'success', True)
            result_holder[0] = success
            self.topo_nav_status = 'arrived' if success else 'failed'
            self.topo_navigating = False
            self._nav_goal_handle = None
            done_event.set()

        self._nav_ac.send_goal_async(
            goal, feedback_callback=self._nav_feedback
        ).add_done_callback(_on_accepted)

        deadline = timeout_sec
        interval = 0.25
        while not done_event.wait(timeout=interval):
            deadline -= interval
            if deadline <= 0:
                self.get_logger().warn(f'_send_goal_sync: timeout for {target}')
                self.cancel_nav_goal()
                return False
            if self._mission_cancel or self.soft_estop_active:
                self.cancel_nav_goal()
                done_event.wait(timeout=2.0)
                return False

        return bool(result_holder[0])

    def cancel_mission(self) -> None:
        """Signal the executor thread to stop after the current row."""
        self._mission_cancel = True
        self.cancel_nav_goal()

    # ── Map archive ───────────────────────────────────────────────────────────

    def archive_and_clear_map(self) -> str:
        """Copy current map file to /workspace/maps/<name>_<N>, then write a
        fresh empty map doc back to the original path and reload it.

        Returns a status string (caller displays it).
        """
        if not self._topo_doc:
            return 'ERROR: no map loaded'

        map_name = self._topo_doc.name
        map_file = f'/workspace/maps/{map_name}'

        # Pick next available archive index
        i = 1
        while os.path.exists(f'{map_file}_{i}'):
            i += 1
        archive_path = f'{map_file}_{i}'

        try:
            # Read from disk so we archive the persisted state, not just memory
            if os.path.exists(map_file):
                on_disk = parse_topo_yaml(map_file)
            else:
                on_disk = copy.deepcopy(self._topo_doc)

            dump_topo_yaml(on_disk, archive_path)

            # Build empty map doc preserving header fields.
            empty_doc = self._topo_doc.clone_empty(map_name)
            dump_topo_yaml(empty_doc, map_file)
            self._topo_doc = empty_doc

            # Republish so topo nav stack sees the cleared map immediately
            try:
                self._topo_map_pub.publish(_topo_to_msg(empty_doc))
            except Exception:
                pass

            self.get_logger().info(
                f'archive_and_clear_map: archived to {archive_path}')
            return f'archived → {os.path.basename(archive_path)}'

        except Exception as e:
            self.get_logger().error(f'archive_and_clear_map failed: {e}')
            return f'ERROR: {e}'

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
                        def _u():
                            d.set_content(f'<span class="dot-{"warn" if getattr(self,a) else "ok"}"></span>')
                        return _u
                    ui.timer(0.2, _mk())
                ui.html('<div class="sec-label mt-3">E-stops</div>')
                for attr, label in [('estop_front_active', 'Front'), ('estop_back_active', 'Rear')]:
                    with ui.row().classes('items-center gap-0'):
                        dot = ui.html('<span class="dot-off"></span>')
                        ui.label(label).classes('text-sm')
                    def _mk2(d=dot, a=attr):
                        def _u():
                            d.set_content(f'<span class="dot-{"warn" if getattr(self,a) else "off"}"></span>')
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
            leaflet = ui.leaflet(center=(9.045094, 77.792024), zoom=18).classes('w-full h-80')
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

                # ── Graph Explorer ───────────────────────────────────────
                _explorer_proc: list = [None]
                _explorer_lbl = ui.label('').classes('text-xs font-mono').style('color:#57606a')
                def _start_explorer():
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
                ui.html(
                    '<a href="https://github.com/nilseuropa/ros2graph_explorer#build--launch"'
                    ' target="_blank"'
                    ' style="font-size:11px;color:var(--txt-muted);text-decoration:none;'
                    'font-family:\'Courier New\',monospace;">'
                    '📄 Documentation</a>'
                )

                ui.separator().classes('w-full my-1')

                # ── ros2grapher ──────────────────────────────────────────
                _grapher_proc: list = [None]
                _grapher_lbl = ui.label('').classes('text-xs font-mono').style('color:#57606a')
                def _start_grapher():
                    if _grapher_proc[0] is not None and _grapher_proc[0].poll() is None:
                        _grapher_lbl.set_text('already running')
                        return
                    try:
                        _grapher_proc[0] = subprocess.Popen(
                            ['ros2grapher', '/workspace'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        )
                        _grapher_lbl.set_text(
                            f'started (pid {_grapher_proc[0].pid}) — '
                            f'open http://localhost:8888')
                        _grapher_lbl.style('color:#1a7f37')
                    except Exception as exc:
                        _grapher_lbl.set_text(f'ERROR: {exc}')
                        _grapher_lbl.style('color:#cf222e')
                ui.button('Start ros2grapher', on_click=_start_grapher).props(
                    'outline no-caps').classes('px-4')
                ui.html(
                    '<a href="http://localhost:8888/" target="_blank" '
                    'style="font-size:13px;color:var(--blue);text-decoration:none;'
                    'padding:6px 12px;border:1px solid var(--blue);border-radius:4px;'
                    'font-family:\'Courier New\',monospace;">'
                    '↗ ros2grapher</a>'
                )
                ui.html(
                    '<a href="https://github.com/Supull/ros2grapher"'
                    ' target="_blank"'
                    ' style="font-size:11px;color:var(--txt-muted);text-decoration:none;'
                    'font-family:\'Courier New\',monospace;">'
                    '📄 Documentation</a>'
                )

                ui.separator().classes('w-full my-1')

                # ── RViz ─────────────────────────────────────────────────
                _rviz_proc: list = [None]
                _rviz_daemons: list = []  # Xvfb, x11vnc, websockify - tracked for clean shutdown
                _rviz_lbl = ui.label('').classes('text-xs font-mono').style('color:#57606a')

                def _start_rviz():
                    if _rviz_proc[0] is not None and _rviz_proc[0].poll() is None:
                        _rviz_lbl.set_text('already running')
                        return
                    try:
                        # Resolve topo_nav RViz config (has MarkerArray displays
                        # pre-wired to /topological_map_visualisation et al).
                        # Falls back to no config if topo_nav isn't installed.
                        try:
                            rviz_cfg = os.path.join(
                                get_package_share_directory('topological_navigation'),
                                'rviz', 'topological_navigation.rviz',
                            )
                        except PackageNotFoundError:
                            rviz_cfg = None

                        # We don't use `with` for these because we save the process arguments and
                        # manage them manually.
                        # Spawn Xvfb only if :98 isn't already taken (re-launch safe).
                        if not os.path.exists('/tmp/.X98-lock'):
                            _rviz_daemons.append(subprocess.Popen(
                                ['Xvfb', ':98', '-screen', '0', '1920x1080x24',
                                 '-nolisten', 'tcp'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            ))
                            time.sleep(0.5)

                        _rviz_daemons.append(subprocess.Popen(
                            ['x11vnc', '-display', ':98', '-nopw', '-forever', '-shared',
                             '-quiet', '-rfbport', '5901'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        ))
                        _rviz_daemons.append(subprocess.Popen(
                            ['websockify', '--web', '/usr/share/novnc', '6081',
                             'localhost:5901'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        ))
                        time.sleep(0.5)

                        rviz_args = ['ros2', 'run', 'rviz2', 'rviz2']
                        if rviz_cfg is not None:
                            rviz_args += ['-d', rviz_cfg]

                        _rviz_proc[0] = subprocess.Popen(
                            rviz_args,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env={**os.environ, 'DISPLAY': ':98'},
                        )
                        suffix = '' if rviz_cfg else ' (no topo config found)'
                        _rviz_lbl.set_text(f'started (pid {_rviz_proc[0].pid}){suffix}')
                        _rviz_lbl.style('color:#1a7f37')
                    except Exception as exc:
                        _rviz_lbl.set_text(f'ERROR: {exc}')
                        _rviz_lbl.style('color:#cf222e')

                def _stop_rviz():
                    if _rviz_proc[0] is not None:
                        _rviz_proc[0].terminate()
                        _rviz_proc[0] = None
                    for p in _rviz_daemons:
                        try:
                            p.terminate()
                        except Exception:
                            pass
                    _rviz_daemons.clear()
                    _rviz_lbl.set_text('stopped')
                    _rviz_lbl.style('color:#57606a')

                ui.button('Launch RViz', on_click=_start_rviz).props(
                    'outline no-caps').classes('px-4')
                ui.button('Stop RViz', on_click=_stop_rviz).props(
                    'outline no-caps').classes('px-4')
                ui.html(
                    '<a href="http://localhost:6081/vnc.html" target="_blank" '
                    'style="font-size:13px;color:var(--blue);text-decoration:none;'
                    'padding:6px 12px;border:1px solid var(--blue);border-radius:4px;'
                    'font-family:\'Courier New\',monospace;">'
                    '↗ RViz (noVNC)</a>'
                )
                ui.separator().classes('w-full my-1')

                # ── Gazebo Sim ───────────────────────────────────────────
                _gazebo_proc: list = [None]
                _spawn_proc: list = [None]
                _gazebo_daemons: list = []   # Xvfb, x11vnc, websockify for browser mode
                _gazebo_lbl = ui.label('').classes('text-xs font-mono').style('color:#57606a')

                # Robot model selector — controls which xacro is spawned and
                # which urdf arg is passed to sowbot_sim.launch.py.
                # sowbot_01:       TrackedVehicle + TrackController (DART required)
                # robo_caatinga:   DiffDrive skid-steer (ODE or DART both fine)
                # ifarmate:        DiffDrive skid-steer (plugin wiring copied from caatinga)
                _ROBOT_MODELS = {
                    'sowbot (tracked)':      'sowbot_01.xacro',
                    'caatinga (diff drive)': 'robo_caatinga.urdf.xacro',
                    'ifarmate (diff drive)': 'ifarmate.urdf.xacro',
                }
                _robot_model: dict = {'xacro': 'sowbot_01.xacro'}

                with ui.row().classes('items-center gap-3 mb-1'):
                    ui.html('<span style="font-size:12px;color:#57606a;'
                            'font-family:monospace">Robot model</span>')
                    ui.toggle(
                        list(_ROBOT_MODELS.keys()),
                        value='sowbot (tracked)',
                        on_change=lambda e: _robot_model.update(
                            xacro=_ROBOT_MODELS[e.value]),
                    ).props('dense')

                _SIM_ENV = {
                    **os.environ,
                    'TMAP2_FILE': '/workspace/maps/maize_map',
                    'GZ_SIM_RESOURCE_PATH': (
                        # Forest3D-generated models (model://ground,
                        # model://crop/plant) live here — must be first or
                        # gz sim aborts world load with "Unable to find uri".
                        '/workspace/models'
                        ':/workspace/install/virtual_maize_field'
                        '/share/virtual_maize_field/models'
                        + (':' + os.environ['GZ_SIM_RESOURCE_PATH']
                           if os.environ.get('GZ_SIM_RESOURCE_PATH') else '')
                    ),
                }
                def _sim_cmd() -> list:
                    """Build the sowbot_sim launch command using the current robot model."""
                    return [
                        'ros2', 'launch', 'devkit_bringup', 'sowbot_sim.launch.py',
                        'world:=maize.world',
                        f'urdf:={_robot_model["xacro"]}',
                    ]

                def _start_gazebo_browser():
                    if _gazebo_proc[0] is not None and _gazebo_proc[0].poll() is None:
                        _gazebo_lbl.set_text('already running')
                        return
                    try:
                        # We don't use `with` for these because we save the process arguments and
                        # manage them manually.
                        # Spawn Xvfb on :99 only if not already taken
                        if not os.path.exists('/tmp/.X99-lock'):
                            p = subprocess.Popen(
                                ['Xvfb', ':99', '-screen', '0', '1920x1080x24',
                                 '-nolisten', 'tcp'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            )
                            _gazebo_daemons.append(p)
                            time.sleep(0.5)
                        _gazebo_daemons.append(subprocess.Popen(
                            ['x11vnc', '-display', ':99', '-nopw', '-forever',
                             '-shared', '-quiet', '-rfbport', '5900'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        ))
                        _gazebo_daemons.append(subprocess.Popen(
                            ['websockify', '--web', '/usr/share/novnc',
                             '6080', 'localhost:5900'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        ))
                        time.sleep(0.5)
                        # Xvfb has no DRI/GLX driver, so it can't honour
                        # whatever hardware-render env manage.py set for the
                        # container (nvidia __GLX_VENDOR_LIBRARY_NAME, or
                        # /dev/dri passthrough). Force llvmpipe software GL
                        # for this process only, matching docs/research/Sim.md
                        #
                        # Env vars alone are NOT enough for gz-sim: RViz's
                        # Ogre1/GLX renderer honours LIBGL_ALWAYS_SOFTWARE
                        # directly, but gz-sim's Ogre2 initialises via
                        # EGL_EXT_platform_device, which explicitly enumerates
                        # /dev/dri and selects a real GPU node — Mesa's
                        # software-force guard refuses to override an
                        # explicitly-selected hardware device (this is the
                        # "Not allowed to force software rendering..." warning
                        # immediately before the segfault in gazebo_sim.log).
                        # No EGL env var changes that once a real render node
                        # is visible, and this container's /dev:/dev +
                        # privileged mode means it always is.
                        #
                        # Confirmed fix (tested manually in-container): hide
                        # /dev/dri from just this subprocess with a private
                        # mount namespace — the same trick gz-sim's own CI
                        # uses on GPU-less runners. Only this child's view of
                        # /dev is masked; nothing else in the container.
                        env = {
                            **_SIM_ENV,
                            'DISPLAY': ':99',
                            'LIBGL_ALWAYS_SOFTWARE': '1',
                            'GALLIUM_DRIVER': 'llvmpipe',
                            'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
                        }
                        _quoted_cmd = ' '.join(
                            "'" + a.replace("'", "'\\''") + "'" for a in _sim_cmd())
                        wrapped_cmd = [
                            'unshare', '--mount', '--propagation', 'private',
                            '--', 'bash', '-c',
                            f'mount -t tmpfs tmpfs /dev/dri 2>/dev/null; '
                            f'exec {_quoted_cmd}',
                        ]
                        # Log to a file (not DEVNULL) so a crashed launch is
                        # diagnosable — tail /tmp/gazebo_sim.log.
                        _gz_log = open('/tmp/gazebo_sim.log', 'w',encoding="utf-8")
                        _gazebo_proc[0] = subprocess.Popen(
                            wrapped_cmd,
                            stdout=_gz_log, stderr=subprocess.STDOUT,
                            env=env,
                            start_new_session=True,
                        )
                        _gazebo_lbl.set_text(
                            f'browser mode — {_robot_model["xacro"]} — pid {_gazebo_proc[0].pid}')
                        _gazebo_lbl.style('color:#1a7f37')
                    except Exception as exc:
                        _gazebo_lbl.set_text(f'ERROR: {exc}')
                        _gazebo_lbl.style('color:#cf222e')

                def _stop_gazebo():
                    for proc_var in (_gazebo_proc, _spawn_proc):
                        if proc_var[0] is not None:
                            try:
                                pgid = os.getpgid(proc_var[0].pid)
                                os.killpg(pgid, signal.SIGTERM)
                                proc_var[0].wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                os.killpg(pgid, signal.SIGKILL)
                                proc_var[0].wait(timeout=2)
                            except Exception:
                                pass
                            proc_var[0] = None
                    for p in _gazebo_daemons:
                        try:
                            p.terminate()
                            p.wait(timeout=3)
                        except Exception:
                            pass
                    _gazebo_daemons.clear()
                    _gazebo_lbl.set_text('stopped')
                    _gazebo_lbl.style('color:#57606a')

                # Rebuild maize.world FROM the saved topo map: plants are
                # studded in the inter-row gaps of the R*_IN/OUT nodes. Gazebo
                # reads the world only at launch, so a running sim must be
                # stopped and relaunched to see a rebuild — we refuse mid-run
                # rather than silently no-op. The generator script is mounted
                # at /workspace/topo_to_forest3d.py by manage.py.
                _MAP_FILE   = '/workspace/maps/maize_map'
                _WORLD_FILE = ('/workspace/install/devkit_simulation/share/'
                               'devkit_simulation/worlds/maize.world')

                def _rebuild_world():
                    if _gazebo_proc[0] is not None and _gazebo_proc[0].poll() is None:
                        _gazebo_lbl.set_text(
                            'stop the sim before rebuilding — Gazebo reads the '
                            'world only at launch')
                        _gazebo_lbl.style('color:#cf222e')
                        return
                    if not os.path.exists(_MAP_FILE):
                        _gazebo_lbl.set_text(
                            f'no saved map at {_MAP_FILE} — drop/save nodes first')
                        _gazebo_lbl.style('color:#cf222e')
                        return
                    try:
                        _gazebo_lbl.set_text('rebuilding world from map…')
                        _gazebo_lbl.style('color:#57606a')
                        # Get plant placement values (cm → m conversion)
                        spacing_m = float(plant_spacing.value or 80) / 100.0
                        row_w_m = float(row_width_input.value or 80) / 100.0
                        scale = float(plant_scale.value or 100) / 100.0
                        cat = scale_category.value or 'all'
                        model = plant_model.value or 'plant'
                        weed_density = int(weed_density_scale.value) or 100 
                        # Validate selected model exists on disk
                        model_dir = _CROP_MODELS_DIR / model
                        if not model_dir.is_dir() or not (model_dir / 'model.sdf').is_file():
                            _gazebo_lbl.set_text(
                                f'model "{model}" not found — upload it first')
                            _gazebo_lbl.style('color:#cf222e')
                            return
                        r = subprocess.run(
                            ['python3', '/workspace/topo_to_forest3d.py',
                             '--topo', _MAP_FILE,
                             '--out', '/workspace/forest3d.yaml',
                             '--generate',
                             '--world-out', _WORLD_FILE,
                             '--models-path', '/workspace/models',
                             '--plant-spacing', str(spacing_m),
                             '--row-width', str(row_w_m),
                             '--plant-scale', str(scale),
                             '--scale-category', cat,
                             '--crop-model', model,
                             '--weed-density', str(weed_density)],
                            capture_output=True, text=True, timeout=120, check=False
                        )
                        if r.returncode != 0:
                            err = (r.stderr or r.stdout or 'unknown error').strip()
                            _gazebo_lbl.set_text(f'rebuild failed: {err[-200:]}')
                            _gazebo_lbl.style('color:#cf222e')
                            return
                        summary = (f'world rebuilt (spacing={spacing_m:.2f}m, '
                                  f'row={row_w_m:.2f}m, '
                                  f'scale={scale:.2f} on {cat}, '
                                  f'weed density={weed_density:.2f}, '

                                  f'model={model})')
                        _gazebo_lbl.set_text(f'{summary} — relaunch to view')
                        _gazebo_lbl.style('color:#1a7f37')
                    except subprocess.TimeoutExpired:
                        _gazebo_lbl.set_text('rebuild timed out')
                        _gazebo_lbl.style('color:#cf222e')
                    except Exception as exc:
                        _gazebo_lbl.set_text(f'ERROR: {exc}')
                        _gazebo_lbl.style('color:#cf222e')

                # Hardcoded install prefix — avoids shelling out to
                # `ros2 pkg prefix` which fails when AMENT_PREFIX_PATH
                # is not set in the UI node's subprocess environment.
                _AGRO_PKG = '/workspace/install/devkit_simulation/share/devkit_simulation'

                def _launch_sim():
                    # Single button, runs the exact same thing as the CLI:
                    # `ros2 launch devkit_bringup sowbot_sim.launch.py
                    #   world:=maize.world urdf:=<selected xacro>`
                    # (same command _sim_cmd() builds for the browser button).
                    # Replaces the old Launch World / Spawn Robot split, which
                    # ran sim.launch.py + nav2_only.launch.py instead — that
                    # path skipped preflight_pkill, fusioncore, and
                    # kill_bootstrap_tfs entirely (all of which only exist in
                    # sowbot_sim.launch.py), causing stale wall-time bootstrap
                    # TFs to fight the real sim-time TF forever and FusionCore
                    # to never run at all. Do not reintroduce that split.
                    if _gazebo_proc[0] is not None and _gazebo_proc[0].poll() is None:
                        _gazebo_lbl.set_text('already running')
                        return
                    try:
                        _gazebo_proc[0] = subprocess.Popen(
                            _sim_cmd(),
                            stdout=open('/tmp/gazebo_sim.log', 'w', encoding="utf-8"),
                            stderr=subprocess.STDOUT,
                            env=_SIM_ENV,
                            start_new_session=True,
                        )
                        _gazebo_lbl.set_text(
                            f'sim launching — {_robot_model["xacro"]} — pid {_gazebo_proc[0].pid}')
                        _gazebo_lbl.style('color:#1a7f37')
                    except Exception as exc:
                        _gazebo_lbl.set_text(f'ERROR: {exc}')
                        _gazebo_lbl.style('color:#cf222e')

                # ── Plant Placement Controls ─────────────────────────────
                # Configure plant spacing, weed density row width, and model before
                # rebuilding. Values are stored locally; --plant-spacing and
                # --row-width are passed to topo_to_forest3d.py on rebuild.
                # Passed as --plant-scale, --weed-density --scale-category, and --crop-model.
                _CROP_MODELS_DIR = Path('/workspace/models/crop')

                def _refresh_crop_models():
                    """List valid crop model subfolders (must have model.sdf)."""
                    models = []
                    if _CROP_MODELS_DIR.exists():
                        for d in sorted(_CROP_MODELS_DIR.iterdir()):
                            if d.is_dir() and (d / 'model.sdf').exists():
                                models.append(d.name)
                    return models if models else ['plant']

                ui.separator().classes('w-full my-2')
                ui.html('<span class="sec-label">Plant Placement</span>')

                # Row 1: Plant Spacing | Row Width
                with ui.row().classes('w-full gap-4 mt-1'):
                    with ui.column().classes('flex-1 gap-0'):
                        ui.html('<div class="sec-label">Plant Spacing</div>')
                        plant_spacing = ui.number(
                            value=80, min=10, max=300, step=5, precision=0,
                            suffix='cm'
                        ).classes('w-full')
                    with ui.column().classes('flex-1 gap-0'):
                        ui.html('<div class="sec-label">Row width</div>')
                        row_width_input = ui.number(
                            value=80, min=20, max=300, step=5, precision=0,
                            suffix='cm'
                        ).classes('w-full')

                spacing_warn_lbl = ui.label('').classes('text-xs').style('color:#9a6700')

                def _check_spacing_warning():
                    rw = float(row_width_input.value or 80)
                    ps = float(plant_spacing.value or 80)
                    if rw >= ps:
                        spacing_warn_lbl.set_text('Warning: row width >= plant spacing')
                    else:
                        spacing_warn_lbl.set_text('')

                row_width_input.on('update:model-value', lambda e: _check_spacing_warning())
                plant_spacing.on('update:model-value', lambda e: _check_spacing_warning())

                # Row 2: Scale + category | Model selector
                with ui.row().classes('w-full gap-4 mt-1'):
                    with ui.column().classes('flex-1 gap-0'):
                        ui.html('<div class="sec-label">Scale</div>')
                        with ui.row().classes('items-center gap-1 w-full'):
                            plant_scale = ui.number(
                                value=100, min=5, max=1000, step=5, precision=0,
                                suffix='%'
                            ).classes('flex-1')
                            ui.label('on').classes('text-xs').style('color:#8c959f')
                            scale_category = ui.select(
                                options=['all', 'crop', 'weed', 'irrigation'],
                                value='all'
                            ).classes('w-28')
                    with ui.column().classes('flex-1 gap-0'):
                        ui.html('<div class="sec-label">Model</div>')
                        plant_model = ui.select(
                            options=_refresh_crop_models(),
                            value=_refresh_crop_models()[0] if _refresh_crop_models() else None
                        ).classes('w-full')

                # Row 3: weed_density + category (coming)
                with ui.row().classes('w-full gap-4 mt-1'):
                    with ui.column().classes('flex-1 gap-0'):
                        ui.html('<div class="sec-label">Weed Density</div>')
                        with ui.row().classes('items-center gap-1 w-full'):
                            weed_density_scale = ui.number(
                                value=100, min=0, max=100, step=5, precision=0,
                                suffix='%'
                            ).classes('flex-1')
                            ui.label('on').classes('text-xs').style('color:#8c959f')

                # Upload section
                ui.html('<div class="sec-label mt-2">Upload new model</div>')
                _visual_mesh_data: dict = {'name': None, 'data': None}
                _collision_mesh_data: dict = {'name': None, 'data': None}
                _plant_upload_lbl = ui.label('').classes('text-xs font-mono').style(
                    'color:#57606a')

                model_name_input = ui.input(
                    label='Model name',
                    placeholder='my_plant',
                ).classes('w-40')

                def _is_valid_gltf(data, fname):
                    ext = fname.lower().rsplit('.', 1)[-1] if '.' in fname else ''
                    if ext == 'glb':
                        return data[:4] == b'glTF'
                    if ext == 'gltf':
                        return data.strip()[:1] in (b'{', b'[')
                    return False

                async def _handle_visual_upload(e):
                    try:
                        if hasattr(e, 'file'):
                            data = await e.file.read()
                            fname = e.file.name if hasattr(e.file, 'name') else 'visual.glb'
                        else:
                            data = e.content.read()
                            fname = getattr(e, 'name', 'visual.glb')
                    except Exception as exc:
                        _plant_upload_lbl.set_text(f'upload failed: {exc}')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    # Validate extension
                    if not fname.lower().endswith(('.glb', '.gltf')):
                        _plant_upload_lbl.set_text('only .glb/.gltf files accepted')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    # Validate magic bytes
                    if not _is_valid_gltf(data, fname):
                        _plant_upload_lbl.set_text('invalid glTF file')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    # Validate size (50MB cap)
                    if len(data) > 50 * 1024 * 1024:
                        _plant_upload_lbl.set_text('file too large (max 50MB)')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    _visual_mesh_data['name'] = fname
                    _visual_mesh_data['data'] = data
                    _plant_upload_lbl.set_text(f'visual: {fname} ({len(data)//1024}KB)')
                    _plant_upload_lbl.style('color:#1a7f37')

                async def _handle_collision_upload(e):
                    try:
                        if hasattr(e, 'file'):
                            data = await e.file.read()
                            fname = e.file.name if hasattr(e.file, 'name') else 'collision.glb'
                        else:
                            data = e.content.read()
                            fname = getattr(e, 'name', 'collision.glb')
                    except Exception as exc:
                        _plant_upload_lbl.set_text(f'collision upload failed: {exc}')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    if not fname.lower().endswith(('.glb', '.gltf')):
                        _plant_upload_lbl.set_text('only .glb/.gltf files accepted')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    if not _is_valid_gltf(data, fname):
                        _plant_upload_lbl.set_text('invalid glTF file')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    if len(data) > 50 * 1024 * 1024:
                        _plant_upload_lbl.set_text('collision file too large (max 50MB)')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    _collision_mesh_data['name'] = fname
                    _collision_mesh_data['data'] = data
                    _plant_upload_lbl.set_text(
                        f'visual: {_visual_mesh_data["name"] or "—"}, '
                        f'collision: {fname}')
                    _plant_upload_lbl.style('color:#1a7f37')

                async def _create_plant_model():
                    name = (model_name_input.value or '').strip()
                    if not name:
                        _plant_upload_lbl.set_text('enter a model name')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    if not re.match(r'^[a-zA-Z0-9_]+$', name):
                        _plant_upload_lbl.set_text('name must be alphanumeric + underscore')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    model_dir = _CROP_MODELS_DIR / name
                    if model_dir.exists():
                        _plant_upload_lbl.set_text(f'model "{name}" already exists')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    if _visual_mesh_data['data'] is None:
                        _plant_upload_lbl.set_text('upload a visual mesh first')
                        _plant_upload_lbl.style('color:#cf222e')
                        return
                    try:
                        mesh_dir = model_dir / 'mesh'
                        mesh_dir.mkdir(parents=True, exist_ok=True)
                        # Write visual mesh
                        visual_fname = f'{name}.glb'
                        with open(mesh_dir / visual_fname, 'wb') as f:
                            f.write(_visual_mesh_data['data'])
                        # Write collision mesh (or reuse visual)
                        if _collision_mesh_data['data'] is not None:
                            collision_fname = f'{name}_collision.glb'
                            with open(mesh_dir / collision_fname, 'wb') as f:
                                f.write(_collision_mesh_data['data'])
                        else:
                            collision_fname = visual_fname
                        # Write model.config
                        model_config = f'''<?xml version="1.0"?>
<model>
    <name>{name}</name>
    <version>1.0</version>
    <sdf version="1.8">model.sdf</sdf>
    <author>
        <name>User Upload</name>
    </author>
    <description>{name} plant model</description>
</model>
'''
                        with open(model_dir / 'model.config', 'w', encoding="utf-8") as f:
                            f.write(model_config)
                        # Write model.sdf
                        model_sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
    <model name="{name}">
        <static>true</static>
        <link name="link">
            <collision name="collision">
                <geometry>
                    <mesh><uri>mesh/{collision_fname}</uri><scale>1 1 1</scale></mesh>
                </geometry>
            </collision>
            <visual name="visual">
                <geometry>
                    <mesh><uri>mesh/{visual_fname}</uri><scale>1 1 1</scale></mesh>
                </geometry>
            </visual>
        </link>
    </model>
</sdf>
'''
                        with open(model_dir / 'model.sdf', 'w', encoding="utf-8") as f:
                            f.write(model_sdf)
                        # Refresh dropdown
                        new_models = _refresh_crop_models()
                        plant_model.options = new_models
                        plant_model.value = name
                        # Clear upload state
                        _visual_mesh_data['name'] = None
                        _visual_mesh_data['data'] = None
                        _collision_mesh_data['name'] = None
                        _collision_mesh_data['data'] = None
                        model_name_input.value = ''
                        _plant_upload_lbl.set_text(f'model "{name}" created')
                        _plant_upload_lbl.style('color:#1a7f37')
                    except Exception as exc:
                        _plant_upload_lbl.set_text(f'failed: {exc}')
                        _plant_upload_lbl.style('color:#cf222e')

                with ui.row().classes('items-center gap-2 flex-wrap'):
                    ui.upload(
                        label='Visual mesh (.glb/.gltf) *',
                        auto_upload=True,
                        on_upload=_handle_visual_upload,
                    ).props('accept=.glb,.gltf').classes('max-w-xs')
                    ui.upload(
                        label='Collision mesh (optional)',
                        auto_upload=True,
                        on_upload=_handle_collision_upload,
                    ).props('accept=.glb,.gltf').classes('max-w-xs')
                    ui.button('Create Model', on_click=_create_plant_model).props(
                        'color=primary no-caps').classes('px-4')

                ui.separator().classes('w-full my-2')

                with ui.row().classes('items-center gap-2 flex-wrap'):
                    ui.button('Launch Sim', on_click=_launch_sim).props('color=positive no-caps').classes('px-4 font-bold')
                    ui.button('Rebuild World from Map', on_click=_rebuild_world).props(
                        'outline no-caps').classes('px-4')
                    ui.button('Launch Sim (browser)', on_click=_start_gazebo_browser).props(
                        'outline no-caps').classes('px-4')
                    ui.button('Stop Sim', on_click=_stop_gazebo).props(
                        'outline no-caps').classes('px-4')
                    ui.html(
                        '<a href="http://localhost:6080/vnc.html" target="_blank" '
                        'style="font-size:13px;color:var(--blue);text-decoration:none;'
                        'padding:6px 12px;border:1px solid var(--blue);border-radius:4px;'
                        'font-family:\'Courier New\',monospace;">'
                        '↗ Gazebo (noVNC)</a>'
                    )


                # ── Sowbot Row Follow (sim) ──────────────────────────────
                # Launches neo.launch.py in sim mode: subscribes to the
                # Gazebo-bridged /camera/image_raw instead of opening a
                # V4L2 device, runs the TSM detector, and publishes
                # /cmd_vel via crop_row_node. limbic_row_follow_node
                # (started by sim_nav.launch.py) calls /row_follow/enable
                # on this process when topo nav reaches an _IN node.
                ui.separator().classes('w-full my-1')
                _neo_proc: list = [None]
                _neo_lbl = ui.label('').classes('text-xs font-mono').style('color:#57606a')

                def _start_neo():
                    if _neo_proc[0] is not None and _neo_proc[0].poll() is None:
                        _neo_lbl.set_text('already running')
                        return
                    try:
                        _neo_proc[0] = subprocess.Popen(
                            [
                                'ros2', 'launch', 'devkit_bringup', 'neo.launch.py',
                                'use_camera:=false',
                                'detector:=tsm',
                                'image_topic:=/camera/image_raw',
                            ],
                            stdout=open('/tmp/neo_sim.log', 'w', encoding="utf-8"),
                            stderr=subprocess.STDOUT,
                            env=os.environ.copy(),
                            start_new_session=True,
                        )
                        _neo_lbl.set_text(f'running — pid {_neo_proc[0].pid} · log: /tmp/neo_sim.log')
                        _neo_lbl.style('color:#1a7f37')
                    except Exception as exc:
                        _neo_lbl.set_text(f'ERROR: {exc}')
                        _neo_lbl.style('color:#cf222e')

                def _stop_neo():
                    if _neo_proc[0] is None:
                        _neo_lbl.set_text('not running')
                        return
                    try:
                        pgid = os.getpgid(_neo_proc[0].pid)
                        os.killpg(pgid, signal.SIGTERM)
                        _neo_proc[0].wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(pgid, signal.SIGKILL)
                        _neo_proc[0].wait(timeout=2)
                    except Exception:
                        pass
                    _neo_proc[0] = None
                    _neo_lbl.set_text('stopped')
                    _neo_lbl.style('color:#57606a')

                with ui.row().classes('items-center gap-2 flex-wrap'):
                    ui.html('<span class=\"sec-label\" style=\"white-space:nowrap\">Sowbot Row Follow</span>')
                    ui.button('Start', on_click=_start_neo).props('color=positive no-caps').classes('px-4')
                    ui.button('Stop',  on_click=_stop_neo).props('color=negative outline no-caps').classes('px-4')

                # ── Soil texture import ──────────────────────────────────
                # Import a soil asset folder (zipped): its image maps are
                # harvested into /workspace/uploads (persisted across image
                # rebuilds) and staged into the ground model on the next world
                # rebuild, where Forest3D turns them into a PBR material.
                _SOIL_TEX_DIR = Path('/workspace/uploads/soil_custom/textures')
                _soil_lbl = ui.label('').classes('text-xs font-mono').style('color:#57606a')

                def _classify_map(name):
                    # Mirror Forest3D's filename-keyword classification so the
                    # label previews what the PBR material will use.
                    nl = name.lower()
                    if any(k in nl for k in ('diff', 'albedo', 'base', 'color')):
                        return 'albedo'
                    if any(k in nl for k in ('normal', 'nor', 'nrm')):
                        return 'normal'
                    if 'rough' in nl:
                        return 'roughness'
                    return 'other'

                def _harvest_soil_zip(data):
                    # Sync worker (runs off the event loop via io_bound): extract
                    # gz-loadable image maps from the zip bytes into _SOIL_TEX_DIR.
                    # Returns the staged basenames; raises BadZipFile / ValueError.
                    zf = zipfile.ZipFile(io.BytesIO(data))
                    # Forest3D skips .exr, so only harvest gz-loadable images.
                    members = [m for m in zf.namelist()
                               if not m.endswith('/')
                               and Path(m).suffix.lower() in ('.jpg', '.jpeg', '.png')]
                    if not members:
                        raise ValueError(
                            'no .jpg/.png maps found in the zip '
                            '(textures may be .exr — convert first)')
                    # Replace any previous import so exactly one soil set is
                    # active; flatten folder structure to basenames.
                    if _SOIL_TEX_DIR.exists():
                        shutil.rmtree(_SOIL_TEX_DIR)
                    _SOIL_TEX_DIR.mkdir(parents=True, exist_ok=True)
                    names = []
                    for m in members:
                        out = _SOIL_TEX_DIR / Path(m).name
                        with zf.open(m) as src, open(out, 'wb') as fh:
                            shutil.copyfileobj(src, fh)
                        names.append(out.name)
                    return names

                async def _import_soil_zip(e):
                    # NiceGUI changed the upload event shape across versions:
                    # newer exposes e.file (FileUpload, async read()); older
                    # exposed e.content (a sync file-like object).
                    try:
                        if hasattr(e, 'file'):
                            data = await e.file.read()
                        else:
                            data = e.content.read()
                    except Exception as exc:
                        _soil_lbl.set_text(f'import failed: {exc}')
                        _soil_lbl.style('color:#cf222e')
                        return
                    try:
                        names = await ng_run.io_bound(_harvest_soil_zip, data)
                    except zipfile.BadZipFile:
                        _soil_lbl.set_text('not a valid .zip file')
                        _soil_lbl.style('color:#cf222e')
                        return
                    except ValueError as exc:
                        _soil_lbl.set_text(str(exc))
                        _soil_lbl.style('color:#cf222e')
                        return
                    except Exception as exc:
                        _soil_lbl.set_text(f'import failed: {exc}')
                        _soil_lbl.style('color:#cf222e')
                        return
                    summary = ', '.join(f'{_classify_map(n)}={n}' for n in names)
                    _soil_lbl.set_text(
                        f'imported {len(names)} map(s) — {summary}. '
                        'Rebuild World to apply.')
                    _soil_lbl.style('color:#1a7f37')

                with ui.row().classes('items-center gap-2 flex-wrap'):
                    ui.upload(
                        label='Import soil asset (.zip)',
                        auto_upload=True,
                        on_upload=_import_soil_zip,
                    ).props('accept=.zip').classes('max-w-md')

        with ui.card().classes('w-full mt-3'):
            ui.label('Map Archive').classes('font-semibold mb-2')
            archive_lbl = ui.label('').classes('text-xs font-mono mt-1').style(
                'color:#57606a')

            async def _do_archive():
                map_name = self._topo_doc.name if self._topo_doc else '?'
                with ui.dialog() as dlg, ui.card():
                    ui.label('Archive and clear map').classes('font-semibold')
                    ui.label(
                        f'Copies "{map_name}" to "{map_name}_N" then wipes all '
                        f'nodes from the live map. Cannot be undone from the UI.'
                    ).classes('text-xs').style('color:#57606a;max-width:340px')
                    with ui.row().classes('w-full justify-end gap-2 mt-3'):
                        ui.button('Cancel',
                                  on_click=lambda: dlg.submit('cancel')).props(
                                      'flat no-caps')
                        ui.button('Archive & Clear', color='negative',
                                  on_click=lambda: dlg.submit('ok')).props('no-caps')

                result = await dlg
                if result != 'ok':
                    return
                status = self.archive_and_clear_map()
                archive_lbl.set_text(status)
                archive_lbl.style(
                    'color:#cf222e' if status.startswith('ERROR')
                    else 'color:#1a7f37')

            ui.button('Archive & Clear Map', on_click=_do_archive).props(
                'color=negative outline no-caps').classes('px-4')

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

    def store_gps(self, msg: NavSatFix) -> None:
        self.latest_gps = msg
        # Anything arriving on the real /gnss/fix topic is by definition a
        # real fix now that the shim publishes elsewhere — no sentinel check
        # needed here any more, but keep the same variable/semantics for the
        # staleness gate below.
        self._last_real_gps_t = self.get_clock().now().nanoseconds * 1e-9

    def store_fake_gps(self, msg: NavSatFix) -> None:
        """Consume the sim shim's fix as a fallback ONLY (see _FAKE_GPS_TOPIC
        setup docstring). This topic is never seen by fusioncore, so this is
        purely for the UI's own use (e.g. the topo-map save path needing a
        finite fix at cold start before the real bridge has published one).
        Content-gated rather than topic-gated: only takes effect if no real
        fix has arrived recently, so a slow-starting real bridge doesn't
        leave the UI without any fix while it comes up.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_real_gps_t < 20.0:
            return  # a real fix was seen recently; don't override it
        self.latest_gps = msg

    def _publish_fake_gps(self) -> None:
        """Publish a fix at the field datum (sim only — timer isn't created
        on hardware). Runs on its own dedicated topic (_FAKE_GPS_TOPIC), so
        there is no shared-topic race with ros_gz_bridge's real navsat
        publisher any more — no discovery-timing backoff needed, since
        fusioncore and the real bridge never see this topic at all.
        """
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = self._FAKE_GPS_SENTINEL
        msg.latitude = self._FAKE_GPS_LAT
        msg.longitude = self._FAKE_GPS_LON
        msg.altitude = self._FAKE_GPS_ALT
        self._fake_gps_pub.publish(msg)

    # pylint: disable=multiple-statements
    def store_battery(self, msg: BatteryState) -> None:      self.latest_battery = msg
    def update_bumper_front_top(self, msg: Bool) -> None:    self.bumper_front_top_active = msg.data
    def update_bumper_front_bottom(self, msg: Bool) -> None: self.bumper_front_bottom_active = msg.data
    def update_bumper_back(self, msg: Bool) -> None:         self.bumper_back_active = msg.data
    def update_estop_front(self, msg: Bool) -> None:         self.estop_front_active = msg.data
    def update_estop_back(self, msg: Bool) -> None:          self.estop_back_active = msg.data
    # pylint: enable=multiple-statements


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
# reload=False is mandatory here. This module is imported (never run as
# __main__) by the `ui_node` console-script entry point, so there's no
# __name__ guard around this call -- every import executes it. NiceGUI's
# default reload=True spins up uvicorn's reload supervisor, which re-imports
# this module in a worker process to load `app`; that re-import re-runs this
# exact line a second time and tries to bind port 80 again while the
# supervisor still holds it -> EADDRINUSE. Hot-reload also has no use case
# in a container that gets rebuilt/restarted on code changes anyway.
ui.run(favicon='🤖', port=80, reload=False)
