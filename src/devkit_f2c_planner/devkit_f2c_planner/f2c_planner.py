"""
f2c_planner.py
──────────────
Field2Cover planning core: lat/lon <-> local-XY projection and the swath
generator. Standalone package (devkit_f2c_planner) with no NiceGUI/UI
dependency, so any caller can use it.

Originally lived inside devkit_ui/ui_node.py, coupled to the manual-drawing
mission workflow. First pulled into devkit_ui/f2c_planner.py so the
terrain-aware pipeline (devkit_ui/terrain_mask.py) could share the same
projection — anchored at the same lat0/lon0 — without reaching into
ui_node's internals. Promoted to its own package because "planning core"
and "web UI" are different concerns with different reasons to change, and
nothing about this module actually depends on devkit_ui.

devkit_ui still owns the manual-drawing UI and calls _run_f2c() the same
way it always did, now via the devkit_f2c_planner dependency instead of a
same-package import.
"""

import math
import sys

# pylint: disable=import-error
import fields2cover as f2c
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import unary_union

# offset_curve() (shapely >= 2.0) is used by _run_contour_f2c() below. The
# repo doesn't pin a shapely version — it's installed generically in the
# Dockerfile, same as fields2cover — so this is flagged here rather than
# discovered as a cryptic AttributeError on an older pin.
_SHAPELY_OFFSET_CURVE_MIN_VERSION = '2.0'


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


def field_centroid_xy(corners_ll: list) -> tuple[float, float]:
    """Field boundary's spatial centroid, in the same local-xy frame
    _run_f2c()/_run_contour_f2c() use (anchored at corners_ll[0]).

    Callers pass this straight through as dem.select_reference_contour_xy()'s
    centroid_xy argument, to get a reference contour centred on the field —
    see that function's docstring for why centring keeps drift symmetric.
    """
    lat0, lon0 = corners_ll[0]
    poly = Polygon([_f2c_latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in corners_ll])
    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly.centroid.x, poly.centroid.y


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
    obstacle_polys_xy = _build_obstacle_polys(
        obstacle_rings, lat0, lon0, obstacle_pad_m, _log, f2c_cell=cell)
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
    if obstacle_polys_xy:
        raw_xy = _clip_lines_against_obstacles(
            raw_xy, obstacle_polys_xy, tool_width * 0.5, _log)

    # ── Project back to lat/lon ──────────────────────────────────────
    result: list = []
    for pts_xy in raw_xy:
        pts_ll = [_f2c_xy_to_latlon(x, y, lat0, lon0) for x, y in pts_xy]
        if len(pts_ll) >= 2:
            result.append(pts_ll)
    _log(f'returning {len(result)} swaths to UI')
    return result


def _build_obstacle_polys(obstacle_rings: list, lat0: float, lon0: float,
                           obstacle_pad_m: float, _log, f2c_cell=None) -> list:
    """Project obstacle_rings (lat/lon) to shapely polygons in local xy,
    repairing invalid geometry and applying padding. Shared by _run_f2c()
    and _run_contour_f2c() so the two swath styles can't drift apart on
    what counts as a valid obstacle.

    If f2c_cell is given (an f2c.Cell), also adds each obstacle as a hole
    ring on it — the "belt and braces" F2C hint described in the OBSTACLE
    comment above _run_f2c(). _run_contour_f2c() passes None here: its
    swaths aren't generated by F2C at all, so there's no F2C cell to hint.
    """
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

        poly = Polygon(pts_xy)
        was_valid = poly.is_valid
        _log(f'obstacle {idx}: shapely poly valid={was_valid} '
             f'empty={poly.is_empty} area={poly.area:.3f}m²')

        if not was_valid:
            poly = poly.buffer(0)
            if poly.is_valid and not poly.is_empty and poly.geom_type == 'Polygon':
                _log(f'obstacle {idx}: REPAIRED via buffer(0)')
            else:
                _log(f'obstacle {idx}: DROPPED (invalid, repair failed)')
                continue

        if obstacle_pad_m > 0:
            poly = poly.buffer(obstacle_pad_m, join_style=2, resolution=8)
            _log(f'obstacle {idx}: after buffer({obstacle_pad_m}m) '
                 f'empty={poly.is_empty} type={poly.geom_type} '
                 f'area={poly.area:.3f}m²')
        if poly.is_empty or poly.geom_type != 'Polygon':
            _log(f'obstacle {idx}: DROPPED (empty or non-Polygon)')
            continue
        obstacle_polys_xy.append(poly)

        if f2c_cell is not None:
            hole = f2c.LinearRing()
            for x, y in reversed(list(poly.exterior.coords)):
                hole.addPoint(f2c.Point(x, y, 0))
            hole.closeRing()
            f2c_cell.addRing(hole)

    return obstacle_polys_xy


def _clip_lines_against_obstacles(raw_xy: list, obstacle_polys_xy: list,
                                   min_fragment_len_m: float, _log) -> list:
    """line.difference(obstacle_union) for every row, dropping fragments
    too short to be worth driving. Shared post-clip step for both swath
    styles — see the OBSTACLE comment above _run_f2c() for why this is
    done in shapely rather than trusted to F2C's own hole handling.
    """
    obstacles_union = unary_union(obstacle_polys_xy)
    _log(f'obstacle union: type={obstacles_union.geom_type} '
         f'area={obstacles_union.area:.3f}m² '
         f'bounds={obstacles_union.bounds}')
    if raw_xy:
        sample = raw_xy[0]
        _log(f'first row: {len(sample)} pts, '
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
            if sub.length > min_fragment_len_m:
                clipped.append(list(sub.coords))
                if intersects:
                    clipped_count += 1
    _log(f'clipped {clipped_count} rows against obstacles, '
         f'{dropped_count} fully dropped, final={len(clipped)}')
    return clipped


# CONTOUR ROWS: curved swaths that follow the terrain instead of F2C's
# single fixed sweep angle.
#
# F2C's own swath generator (SG_BruteForce, used by _run_f2c() above) only
# produces straight parallel lines — its documented model assumes flat
# topography. That's not a config flag to change, it's the geometry core,
# so this doesn't patch F2C; it's a second, independent row generator that
# reuses _run_f2c()'s projection and obstacle-clipping helpers instead of
# F2C's swath math.
#
# Method: offset a single reference line (dem.select_reference_contour_*(),
# the elevation isoline through the field centroid — see that module for
# why the centroid) by +/- n*tool_width using shapely's offset_curve(),
# both directions, until the offset runs off the field. This gives exact,
# constant row spacing everywhere — the trade-off, discussed and accepted
# rather than avoided, is that only the reference row itself is a true
# elevation contour; rows further out are geometrically parallel to it but
# will drift from the *actual* isoline at that offset, in proportion to
# how much the terrain curves between the two. Centring the reference on
# the field means that drift is at most ~half the field's cross-slope
# width in either direction rather than the full width.
#
# KNOWN LIMITATION, not yet handled: offset_curve() on a bendy reference
# line at a distance approaching the local radius of curvature can produce
# self-intersecting loops ("horns"), same failure mode any offset-curve
# algorithm has. Rows where this happens are logged and dropped rather
# than emitted looped/garbled — flagged as follow-up work once there's
# real field DEM data to know how often this actually bites, rather than
# guessed at now.
def _run_contour_f2c(corners_ll: list,
                      obstacle_rings: list,
                      reference_line_ll: list,
                      tool_width: float,
                      obstacle_pad_m: float = 0.0,
                      headland_width_m: float = 0.0,
                      snake_order: bool = True,
                      max_rows_each_side: int = 500) -> list:

    def _log(msg):
        print(f'[F2C-contour] {msg}', file=sys.stderr, flush=True)

    _log(f'called: {len(corners_ll)} boundary pts, '
         f'{len(obstacle_rings)} obstacles, {len(reference_line_ll)} '
         f'reference pts, pad={obstacle_pad_m}m, headland={headland_width_m}m, '
         f'snake={snake_order}, width={tool_width}m')

    lat0, lon0 = corners_ll[0]

    field_poly = Polygon(
        [_f2c_latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in corners_ll])
    if not field_poly.is_valid:
        field_poly = field_poly.buffer(0)

    swath_poly = field_poly
    if headland_width_m > 0:
        inset = field_poly.buffer(-headland_width_m, join_style=2)
        if not inset.is_empty and inset.geom_type == 'Polygon':
            swath_poly = inset
            _log(f'headland: inset {headland_width_m}m ok')
        else:
            _log(f'headland: inset {headland_width_m}m emptied the field '
                 f'(too wide?) — using full boundary')

    obstacle_polys_xy = _build_obstacle_polys(
        obstacle_rings, lat0, lon0, obstacle_pad_m, _log, f2c_cell=None)

    ref_xy = [_f2c_latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in reference_line_ll]
    if len(ref_xy) < 2:
        _log(f'reference line has {len(ref_xy)} pts, need >= 2 — returning no rows')
        return []
    ref_line = LineString(ref_xy)

    # ── Offset outward from the reference line until we run off the field ──
    def _row_at(offset_m: float) -> list | None:
        if offset_m == 0:
            line = ref_line
        else:
            try:
                line = ref_line.offset_curve(offset_m, join_style=2)
            except Exception as e:
                _log(f'offset_curve({offset_m:.1f}m) failed: {e}')
                return None
        if line.is_empty:
            return None
        if not line.is_simple:
            _log(f'offset {offset_m:.1f}m: self-intersecting ("horn") — dropped, '
                 f'see KNOWN LIMITATION in the module docstring')
            return None
        clipped = line.intersection(swath_poly)
        if clipped.is_empty:
            return None
        return list(clipped.coords) if clipped.geom_type == 'LineString' else None

    rows_by_offset: dict = {}
    center = _row_at(0.0)
    if center is not None:
        rows_by_offset[0.0] = center

    for direction in (1, -1):
        for n in range(1, max_rows_each_side + 1):
            offset_m = direction * n * tool_width
            row = _row_at(offset_m)
            if row is None:
                _log(f'direction {direction:+d}: ran out at offset '
                     f'{offset_m - direction * tool_width:.1f}m ({n - 1} rows)')
                break
            rows_by_offset[offset_m] = row

    raw_xy = [rows_by_offset[k] for k in sorted(rows_by_offset)]
    _log(f'generated {len(raw_xy)} contour rows before obstacle clipping')

    # ── Snake ordering, same as _run_f2c() ────────────────────────────
    if snake_order and raw_xy:
        raw_xy = [list(reversed(pts)) if i % 2 == 1 else list(pts)
                  for i, pts in enumerate(raw_xy)]

    # ── Post-clip against obstacle union ──────────────────────────────
    if obstacle_polys_xy:
        raw_xy = _clip_lines_against_obstacles(
            raw_xy, obstacle_polys_xy, tool_width * 0.5, _log)

    result: list = []
    for pts_xy in raw_xy:
        pts_ll = [_f2c_xy_to_latlon(x, y, lat0, lon0) for x, y in pts_xy]
        if len(pts_ll) >= 2:
            result.append(pts_ll)
    _log(f'returning {len(result)} contour rows to UI')
    return result
