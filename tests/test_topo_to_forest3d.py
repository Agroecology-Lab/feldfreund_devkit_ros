"""Regression tests for topo_to_forest3d.py: plants follow the topo row paths."""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import topo_to_forest3d as t  # noqa: E402


def _node(name, x, y, rid, role, edges=()):
    """Build a node dict shaped like load_topo() output."""
    return {'name': name, 'x': x, 'y': y, 'edges': list(edges), 'row_id': rid,
            'row_role': role, 'gps_lat': None, 'gps_lon': None}


def _row(rid, path):
    return {'rid': rid, 'a': path[0], 'b': path[-1], 'path': path,
            'gps_lat': None, 'gps_lon': None, 'gps_x': None, 'gps_y': None}


def _rot(p, deg):
    th = math.radians(deg)
    return (p[0] * math.cos(th) - p[1] * math.sin(th),
            p[0] * math.sin(th) + p[1] * math.cos(th))


def _dist_to_path(p, path):
    best = float('inf')
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        u = 0 if length2 == 0 else max(0, min(1, (
            (p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
        best = min(best, math.hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy))
    return best


def test_extract_rows_follows_edge_chain_through_waypoints():
    """The row path is entry -> waypoints -> exit in edge order, not file order."""
    nodes = {n['name']: n for n in [
        _node('R1_W2', 2, 2, 1, 'waypoint', ['R1_OUT']),
        _node('R1_OUT', 3, 0, 1, 'exit'),
        _node('R1_IN', 0, 0, 1, 'entry', ['R1_W1', 'R9_OUT']),
        _node('R1_W1', 1, 1, 1, 'waypoint', ['R1_W2']),
    ]}
    (row,) = t.extract_rows(nodes)
    assert row['path'] == [(0, 0), (1, 1), (2, 2), (3, 0)]


def test_extract_rows_falls_back_to_waypoint_number_when_chain_broken():
    """Missing edges still give a correctly ordered path (W2 before W10)."""
    nodes = {n['name']: n for n in [
        _node('R1_IN', 0, 0, 1, 'entry'),
        _node('R1_W10', 10, 0, 1, 'waypoint'),
        _node('R1_W2', 2, 0, 1, 'waypoint'),
        _node('R1_OUT', 12, 0, 1, 'exit'),
    ]}
    (row,) = t.extract_rows(nodes)
    assert [p[0] for p in row['path']] == [0, 2, 10, 12]


def test_resample_path_follows_corner_and_keeps_end_points():
    """Samples sit on both legs of an L-shaped path and include both ends."""
    pts = t.resample_path([(0, 0), (4, 0), (4, 4)], 1.0)
    assert pts[0][:2] == (0, 0)
    assert pts[-1][:2] == pytest.approx((4, 4))
    assert len(pts) == 9
    assert all(_dist_to_path(p[:2], [(0, 0), (4, 0), (4, 4)]) < 1e-9 for p in pts)


def test_order_rows_across_is_orientation_independent():
    """Middle-strip ordering works for rows at 33 degrees, not just X/Y aligned."""
    rows = [_row(i, [_rot((x, 1.5 * i), 33) for x in (0, 10)]) for i in range(6)]
    shuffled = [rows[i] for i in (3, 0, 5, 2, 4, 1)]
    assert [r['rid'] for r in t.order_rows_across(shuffled)] in (
        [0, 1, 2, 3, 4, 5], [5, 4, 3, 2, 1, 0])


def test_compute_field_params_covers_tilted_curved_rows():
    """Terrain box and offset come from every path point, at any bearing."""
    rows = [_row(1, [(2, 3), (8, 9), (14, 4)]), _row(2, [(3, 0), (9, -4)])]
    params, _, _, _, density, offset = t.compute_field_params(rows, 5.0, 1.3, 1.0)
    assert offset == (8.0, 2.5)
    assert params['field_length'] == pytest.approx(12 + 10)
    assert params['field_width'] == pytest.approx(13 + 10)
    assert density > 0


def test_place_plants_puts_every_crop_on_the_row_path(tmp_path):
    """Crops land on the authored polyline; Forest3D's own plants are dropped."""
    world = tmp_path / 'w.world'
    world.write_text(
        '<sdf><world name="w">'
        '<include><uri>model://ground</uri><pose>0 0 0 0 0 0</pose></include>'
        '<include><uri>model://crop/old</uri><name>crop_0</name>'
        '<pose>9 9 0 0 0 0</pose></include>'
        '</world></sdf>')
    path = [(0, 0), (6, 4), (6, 10)]
    n = t.place_plants_along_rows(str(world), [_row(1, path)], 0.5, 'plant',
                                  weed_pct=0, seed=1)
    root = ET.parse(world).getroot().find('world')
    uris = [i.findtext('uri') for i in root.findall('include')]
    assert uris.count('model://ground') == 1
    assert 'model://crop/old' not in uris
    crops = [tuple(float(v) for v in i.findtext('pose').split()[:2])
             for i in root.findall('include')
             if i.findtext('uri') == 'model://crop/plant']
    assert len(crops) == n > 10
    assert max(_dist_to_path(c, path) for c in crops) < 0.05


def test_place_plants_is_deterministic(tmp_path):
    """The same inputs write byte-identical worlds (worldgen.sh caches on inputs)."""
    base = '<sdf><world name="w"/></sdf>'
    out = []
    for name in ('a', 'b'):
        w = tmp_path / f'{name}.world'
        w.write_text(base)
        t.place_plants_along_rows(str(w), [_row(1, [(0, 0), (5, 5)])], 1.0,
                                  'plant', weed_pct=50)
        out.append(w.read_text())
    assert out[0] == out[1]
