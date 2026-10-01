"""Validate and parse a swath plan exported by the web planner (web/index.html).

Pure Python, no ROS or UI imports, so it is unit-testable. The web planner
exports geometry only; topo nodes are built on the robot by
NiceGuiNode.save_f2c_rows_to_topo() because node coordinates depend on the
robot's live GPS/odometry anchor and the loaded topo map.

Schema (version 1): GeoJSON FeatureCollection, [lon, lat] coordinates.
  properties: version, generator, origin [lat, lon], tool_width, angle,
              headland, snake
  features:   properties.role == 'row', LineString, properties.row (int),
              properties.frag (int). Feature order is mission order.
              'field' / 'obstacle' features are informational and ignored.
"""
import json
import math
from typing import NamedTuple

SCHEMA_VERSION = 1
MAX_ROWS = 2000
MAX_POINTS_PER_ROW = 5000
MAX_BYTES = 5_000_000
DEFAULT_MAX_DISTANCE_M = 2000.0


class PlanError(ValueError):
    """The plan file is malformed or unsafe to import."""


class Plan(NamedTuple):
    swaths: list          # [[(lat, lon), ...], ...] in mission order
    origin_ll: tuple      # (lat, lon): the planner's projection origin
    break_after: set      # swath indices with no headland edge to the next swath
    params: dict


def _num(v, what):
    """Return a finite numeric value as a float, or raise PlanError labeled with what."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise PlanError(f'{what}: not a finite number')
    return float(v)


def _latlon(lat, lon, what):
    """Return validated latitude/longitude degrees, or raise PlanError for invalid values."""
    lat, lon = _num(lat, what), _num(lon, what)
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise PlanError(f'{what}: coordinate out of range')
    return lat, lon


def _dist_m(a, b):
    """Estimate distance in metres between two (latitude, longitude) pairs in degrees."""
    r = 6_378_137.0
    dx = math.radians(b[1] - a[1]) * r * math.cos(math.radians(a[0]))
    dy = math.radians(b[0] - a[0]) * r
    return math.hypot(dx, dy)


def parse_plan(text, robot_ll=None, max_distance_m=DEFAULT_MAX_DISTANCE_M) -> Plan:
    """Parse and validate plan text. Raises PlanError with a user-readable reason."""
    if len(text) > MAX_BYTES:
        raise PlanError('file too large')
    try:
        doc = json.loads(text)
    except ValueError as e:
        raise PlanError(f'not valid JSON: {e}') from e
    if not isinstance(doc, dict) or doc.get('type') != 'FeatureCollection':
        raise PlanError('not a GeoJSON FeatureCollection')
    props = doc.get('properties')
    if not isinstance(props, dict):
        raise PlanError('missing top-level properties')
    if props.get('version') != SCHEMA_VERSION:
        raise PlanError(f"unsupported plan version {props.get('version')!r}")
    origin = props.get('origin')
    if not (isinstance(origin, list) and len(origin) == 2):
        raise PlanError('origin must be [lat, lon]')
    origin_ll = _latlon(origin[0], origin[1], 'origin')

    swaths, rows = [], []
    for i, f in enumerate(doc.get('features') or []):
        fp = (f or {}).get('properties') or {}
        if fp.get('role') != 'row':
            continue
        geom = f.get('geometry') or {}
        if geom.get('type') != 'LineString':
            raise PlanError(f'feature {i}: row is not a LineString')
        coords = geom.get('coordinates')
        if not isinstance(coords, list) or not 2 <= len(coords) <= MAX_POINTS_PER_ROW:
            raise PlanError(f'feature {i}: bad point count')
        row = fp.get('row')
        if isinstance(row, bool) or not isinstance(row, int) or row < 0:
            raise PlanError(f'feature {i}: row must be a non-negative integer')
        # GeoJSON is [lon, lat]; the planner and topo code use (lat, lon).
        swaths.append([_latlon(c[1], c[0], f'feature {i}') for c in coords])
        rows.append(row)
        if len(swaths) > MAX_ROWS:
            raise PlanError(f'more than {MAX_ROWS} rows')
    if not swaths:
        raise PlanError('no rows in plan')

    # Fragments of one row (split by an obstacle) must not be joined by a
    # headland edge: that edge would cross the obstacle.
    break_after = {i for i in range(len(rows) - 1) if rows[i] == rows[i + 1]}

    if robot_ll is not None:
        pts = [p for s in swaths for p in (s[0], s[-1])]
        centre = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))
        d = _dist_m(robot_ll, centre)
        if d > max_distance_m:
            raise PlanError(f'plan is {d / 1000:.1f} km from the robot '
                            f'(limit {max_distance_m / 1000:.1f} km); refusing to import')

    params = {k: props.get(k) for k in ('tool_width', 'angle', 'headland', 'snake', 'generator')}
    return Plan(swaths, origin_ll, break_after, params)
