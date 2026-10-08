"""Geometry boundaries for authored row paths and generated terrain."""

import math

import pytest

import topo_to_forest3d as t


@pytest.mark.parametrize('path', [None, []])
def test_row_path_falls_back_to_legacy_endpoints(path):
    row = {'a': (1, 2), 'b': (3, 4), 'path': path}
    assert t.row_path(row) == [(1, 2), (3, 4)]
    del row['path']
    assert t.row_path(row) == [(1, 2), (3, 4)]


def test_row_path_retains_authored_detour():
    row = _row([(1, 2), (-4, 9), (3, 4)])
    assert t.row_path(row) == [(1, 2), (-4, 9), (3, 4)]


@pytest.mark.parametrize('path,expected', [
    ([], 0), ([(2, 3)], 0), ([(2, 3), (2, 3)], 0),
    ([(0, 0), (3, 4), (3, 4), (6, 0)], 10),
])
def test_path_length_measures_arc_including_repeated_points(path, expected):
    assert t.path_length(path) == pytest.approx(expected)


@pytest.mark.parametrize('path', [[(2, -3)], [(2, -3)] * 3])
def test_resample_stationary_path_returns_one_finite_sample(path):
    assert t.resample_path(path, 0.5) == [(2, -3, 0.0)]


def test_resample_stretches_intervals_evenly_and_preserves_reverse_direction():
    samples = t.resample_path([(10, 4), (0, 4)], 3)
    assert len(samples) == 4
    assert [s[0] for s in samples] == pytest.approx([10, 20 / 3, 10 / 3, 0])
    assert [s[1] for s in samples] == [4] * 4
    assert [s[2] for s in samples] == pytest.approx([math.pi] * 4)


def test_resample_uses_arc_distance_across_unequal_segments():
    samples = t.resample_path([(0, 0), (1, 0), (1, 5)], 2)
    assert len(samples) == 4
    for sample, expected in zip(samples, [(0, 0, 0), (1, 1, math.pi / 2),
                                          (1, 3, math.pi / 2), (1, 5, math.pi / 2)], strict=True):
        assert sample == pytest.approx(expected)


def test_resample_skips_duplicate_vertices_without_duplicate_plants():
    samples = t.resample_path([(0, 0), (0, 0), (2, 0), (2, 0), (2, 2), (2, 2)], 1)
    assert [s[:2] for s in samples] == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
    assert all(math.isfinite(v) for s in samples for v in s)


@pytest.mark.parametrize('angle', [0, 33, 90, 135])
def test_order_rows_aligns_opposite_directions_despite_along_row_offsets(angle):
    theta = math.radians(angle)

    def rotate(x, y):
        return (x * math.cos(theta) - y * math.sin(theta),
                x * math.sin(theta) + y * math.cos(theta))

    rows = []
    for i in range(6):
        path = [rotate(i * 20 + x, i * 2) for x in (0, 10)]
        rows.append(_row(path if i % 2 == 0 else path[::-1], i))
    shuffled = [rows[i] for i in (2, 5, 0, 3, 1, 4)]
    original_order = [r['rid'] for r in shuffled]
    ordered = t.order_rows_across(shuffled)
    assert [r['rid'] for r in ordered] == list(range(6))
    assert [r['rid'] for r in shuffled] == original_order


def test_order_rows_uses_waypoints_when_centroids_differ_from_endpoint_midpoints():
    bowed = _row([(0, 0), (5, 12), (10, 0)], 1)
    straight = _row([(0, 2), (10, 2)], 2)
    assert [r['rid'] for r in t.order_rows_across([bowed, straight])] == [2, 1]


def test_order_rows_handles_empty_and_stationary_rows():
    assert t.order_rows_across([]) == []
    rows = [_row([(3, 4)], 1), _row([(1, 2)], 2)]
    assert t.order_rows_across(rows) == rows


def test_field_counts_plants_per_path_including_corners_and_stationary_rows():
    rows = [_row([(10, 20), (13, 20), (13, 24)]),
            _row([(11, 21), (13, 21)], 2), _row([(12, 22)], 3)]
    params, _, _, average, density, offset = t.compute_field_params(rows, 2, 0.9, 1)
    assert (average, density) == (4, 12)  # Eight, three, and one crop respectively.
    assert (params['field_length'], params['field_width']) == (7, 8)
    assert offset == (11.5, 22)
    assert params['num_rows'] == 1


def test_field_clamps_negative_headland_and_schema_minima():
    params, _, _, average, density, offset = t.compute_field_params(
        [_row([(3, -4)])], -2, 0, 1, min_furrow_width=0)
    assert (params['field_length'], params['field_width']) == (5, 5)
    assert params['headland_width'] == 0
    assert (params['row_width'], params['furrow_width']) == (0.2, 0.1)
    assert params['resolution'] == 0.1
    assert (average, density, offset) == (1, 1, (3, -4))


@pytest.mark.parametrize('width,expected_resolution', [(0.2, 0.1), (0.5, 0.2), (2, 0.25)])
def test_small_field_resolution_tracks_row_width(width, expected_resolution):
    params, *_ = t.compute_field_params([_row([(0, 0), (10, 0)])], 2, width, 1)
    assert params['resolution'] == pytest.approx(expected_resolution)


@pytest.mark.parametrize('endpoint', [(1000, 0), (0, 1000)])
def test_large_field_resolution_limits_each_grid_dimension(endpoint):
    params, *_ = t.compute_field_params([_row([(0, 0), endpoint])], 2, 0.9, 10)
    assert params['resolution'] == pytest.approx(2.51)
    assert int(max(params['field_length'], params['field_width']) / params['resolution']) <= 400


@pytest.mark.parametrize('extent,dimensions,offset', [
    ((100, 120, -40, -10), (20, 30), (-95, 48)),
    ((100, 110, -40, -30), (10, 10), (-90, 58)),
])
def test_custom_mesh_centres_on_paths_and_accepts_exact_fit(extent, dimensions, offset):
    row = _row([(10, 20), (15, 28), (20, 18)])
    params, _, _, _, _, shift = t.compute_field_params([row], 100, 0.9, 1, ground_extent=extent)
    assert (params['field_length'], params['field_width']) == dimensions
    assert params['headland_width'] == 0
    assert shift == offset


@pytest.mark.parametrize('extent', [(0, 9.99, 0, 10), (0, 10, 0, 9.99)])
def test_custom_mesh_rejects_overflow_in_either_dimension(extent):
    with pytest.raises(SystemExit, match='mesh cannot contain this row layout'):
        t.compute_field_params([_row([(0, 0), (10, 10), (0, 1)])], 0, 0.9, 1, ground_extent=extent)


def test_field_requires_at_least_one_row():
    with pytest.raises(SystemExit, match='need at least 1 row'):
        t.compute_field_params([], 2, 0.9, 1)


def _row(path, rid=1):
    return {'rid': rid, 'a': path[0], 'b': path[-1], 'path': path,
            'gps_lat': None, 'gps_lon': None, 'gps_x': None, 'gps_y': None}
