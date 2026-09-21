"""Unit tests for obstacle polygon building and swath clipping (shapely only, no fields2cover)."""
# pylint: disable=protected-access
import unittest

from shapely.geometry import Polygon

from devkit_f2c_planner.f2c_test_helpers import LAT0, LON0, planner, to_ll


def square(x0: float, y0: float, size: float) -> list:
    return [(x0, y0), (x0 + size, y0), (x0 + size, y0 + size), (x0, y0 + size)]


def build(rings_xy: list, pad_m: float = 0.0) -> list:
    """Run _build_obstacle_polys() on rings given in local xy metres."""
    return planner._build_obstacle_polys([to_ll(ring) for ring in rings_xy], LAT0, LON0, pad_m, lambda _msg: None)


def clip(rows: list, obstacles: list, min_fragment_len_m: float = 0.5) -> list:
    """Run _clip_lines_against_obstacles() with obstacles given as xy rectangles."""
    return planner._clip_lines_against_obstacles(
        rows, [Polygon(ring) for ring in obstacles], min_fragment_len_m, lambda _msg: None)


class BuildObstaclePolysTest(unittest.TestCase):

    def test_valid_ring_becomes_polygon_with_matching_area(self):
        polys = build([square(0, 0, 10)])
        self.assertEqual(len(polys), 1)
        self.assertEqual(polys[0].geom_type, 'Polygon')
        self.assertAlmostEqual(polys[0].area, 100.0, places=3)

    def test_rings_with_fewer_than_three_points_are_skipped(self):
        polys = build([[(0, 0), (5, 5)], square(20, 20, 10)])
        self.assertEqual(len(polys), 1)
        self.assertAlmostEqual(polys[0].bounds[0], 20.0, places=3)

    def test_zero_area_ring_is_dropped(self):
        self.assertEqual(build([[(0, 0), (5, 0), (10, 0)]]), [])

    def test_self_intersecting_ring_is_repaired_or_dropped_never_returned_invalid(self):
        polys = build([[(0, 0), (10, 10), (10, 0), (0, 10)]])
        for poly in polys:
            self.assertTrue(poly.is_valid)
            self.assertEqual(poly.geom_type, 'Polygon')

    def test_ring_with_a_zero_width_spike_is_repaired_not_dropped(self):
        """A dropped obstacle means swaths run straight through it."""
        spiked_square = [(0, 0), (10, 0), (10, 10), (5, 10), (5, 15), (5, 10), (0, 10)]
        (poly,) = build([spiked_square])
        self.assertTrue(poly.is_valid)
        self.assertAlmostEqual(poly.area, 100.0, places=3)

    def test_bad_ring_does_not_affect_the_others(self):
        polys = build([square(0, 0, 10), [(0, 0), (5, 0), (10, 0)], square(50, 50, 10)])
        self.assertEqual(len(polys), 2)
        self.assertAlmostEqual(polys[0].bounds[0], 0.0, places=3)
        self.assertAlmostEqual(polys[1].bounds[0], 50.0, places=3)

    def test_padding_grows_the_polygon_by_the_pad_on_every_side(self):
        """Mitre joins: a 10 m square padded by 1 m is exactly 12 m x 12 m."""
        (poly,) = build([square(0, 0, 10)], pad_m=1.0)
        self.assertAlmostEqual(poly.area, 144.0, places=3)
        for actual, expected in zip(poly.bounds, (-1.0, -1.0, 11.0, 11.0), strict=True):
            self.assertAlmostEqual(actual, expected, places=3)

    def test_no_padding_leaves_the_polygon_unchanged(self):
        (poly,) = build([square(0, 0, 10)], pad_m=0.0)
        self.assertAlmostEqual(poly.area, 100.0, places=3)


class ClipLinesAgainstObstaclesTest(unittest.TestCase):
    OBSTACLE = square(4, -1, 2)  # spans x 4..6, y -1..1

    def test_row_missing_the_obstacle_is_unchanged(self):
        row = [(0.0, 5.0), (10.0, 5.0)]
        self.assertEqual(clip([row], [self.OBSTACLE]), [row])

    def test_row_through_the_obstacle_is_split_at_its_edges(self):
        result = clip([[(0.0, 0.0), (10.0, 0.0)]], [self.OBSTACLE])
        self.assertEqual(result, [[(0.0, 0.0), (4.0, 0.0)], [(6.0, 0.0), (10.0, 0.0)]])

    def test_row_fully_inside_the_obstacle_is_dropped(self):
        self.assertEqual(clip([[(4.2, 0.0), (5.8, 0.0)]], [self.OBSTACLE]), [])

    def test_fragments_keep_the_direction_of_the_input_row(self):
        """Snake ordering reverses rows before clipping and relies on this."""
        result = clip([[(10.0, 0.0), (0.0, 0.0)]], [self.OBSTACLE])
        self.assertEqual(result, [[(10.0, 0.0), (6.0, 0.0)], [(4.0, 0.0), (0.0, 0.0)]])

    def test_fragments_shorter_than_the_minimum_are_dropped(self):
        obstacle = [(1, -1), (9.5, -1), (9.5, 1), (1, 1)]  # leaves 1.0 m and 0.5 m stubs
        result = clip([[(0.0, 0.0), (10.0, 0.0)]], [obstacle], min_fragment_len_m=0.8)
        self.assertEqual(result, [[(0.0, 0.0), (1.0, 0.0)]])

    def test_fragment_exactly_at_the_minimum_length_is_dropped(self):
        obstacle = [(1, -1), (9.5, -1), (9.5, 1), (1, 1)]
        self.assertEqual(clip([[(0.0, 0.0), (10.0, 0.0)]], [obstacle], min_fragment_len_m=1.0), [])

    def test_overlapping_obstacles_act_as_their_union(self):
        obstacles = [square(4, -1, 2), [(5, -1), (8, -1), (8, 1), (5, 1)]]
        result = clip([[(0.0, 0.0), (10.0, 0.0)]], obstacles)
        self.assertEqual(result, [[(0.0, 0.0), (4.0, 0.0)], [(8.0, 0.0), (10.0, 0.0)]])

    def test_each_row_is_clipped_independently(self):
        rows = [[(0.0, 0.0), (10.0, 0.0)], [(0.0, 5.0), (10.0, 5.0)], [(10.0, 0.5), (0.0, 0.5)]]
        result = clip(rows, [self.OBSTACLE])
        self.assertEqual(len(result), 5)
        self.assertIn([(0.0, 5.0), (10.0, 5.0)], result)


if __name__ == '__main__':
    unittest.main()
