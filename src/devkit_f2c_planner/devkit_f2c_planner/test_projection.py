"""Unit tests for the lat/lon <-> local-xy projection and field_centroid_xy()."""
# pylint: disable=protected-access
import math
import unittest

from devkit_f2c_planner.f2c_test_helpers import planner, silence_planner_log, to_ll

# Anchors: UK, southern hemisphere / western longitude, equator, Australia.
ANCHORS = [(55.0, -1.5), (-9.4, -40.5), (0.0, 0.0), (-33.9, 151.2)]

# Length of one degree of arc on the sphere of radius 6_378_137 m used by the planner.
METRES_PER_DEGREE = 111_319.49


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float, radius: float = 6_378_137.0) -> float:
    lat1, lon1, lat2, lon2 = map(math.radians, (lat1, lon1, lat2, lon2))
    a = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(a))


class ProjectionTest(unittest.TestCase):

    def test_anchor_maps_to_origin(self):
        for lat0, lon0 in ANCHORS:
            with self.subTest(anchor=(lat0, lon0)):
                self.assertEqual(planner._f2c_latlon_to_xy(lat0, lon0, lat0, lon0), (0.0, 0.0))

    def test_one_degree_north_is_one_degree_of_arc_at_any_latitude(self):
        for lat0, lon0 in ANCHORS:
            with self.subTest(anchor=(lat0, lon0)):
                x, y = planner._f2c_latlon_to_xy(lat0 + 1, lon0, lat0, lon0)
                self.assertEqual(x, 0.0)
                self.assertAlmostEqual(y, METRES_PER_DEGREE, delta=0.01)

    def test_east_is_scaled_by_cosine_of_anchor_latitude(self):
        for lat0, cos_lat0 in [(0.0, 1.0), (60.0, 0.5), (-60.0, 0.5)]:
            with self.subTest(lat0=lat0):
                x, y = planner._f2c_latlon_to_xy(lat0, 1.0, lat0, 0.0)
                self.assertEqual(y, 0.0)
                self.assertAlmostEqual(x, METRES_PER_DEGREE * cos_lat0, delta=0.01)

    def test_axes_point_east_and_north(self):
        lat0, lon0 = 55.0, -1.5
        x, y = planner._f2c_latlon_to_xy(lat0 + 0.001, lon0 + 0.001, lat0, lon0)
        self.assertGreater(x, 0)
        self.assertGreater(y, 0)
        x, y = planner._f2c_latlon_to_xy(lat0 - 0.001, lon0 - 0.001, lat0, lon0)
        self.assertLess(x, 0)
        self.assertLess(y, 0)

    def test_roundtrip_latlon_xy_latlon(self):
        for lat0, lon0 in ANCHORS:
            for dlat, dlon in [(0.01, 0.01), (-0.01, 0.02), (0.005, -0.015), (-0.02, -0.02)]:
                with self.subTest(anchor=(lat0, lon0), offset=(dlat, dlon)):
                    x, y = planner._f2c_latlon_to_xy(lat0 + dlat, lon0 + dlon, lat0, lon0)
                    lat, lon = planner._f2c_xy_to_latlon(x, y, lat0, lon0)
                    self.assertAlmostEqual(lat, lat0 + dlat, places=9)
                    self.assertAlmostEqual(lon, lon0 + dlon, places=9)

    def test_roundtrip_xy_latlon_xy(self):
        for lat0, lon0 in ANCHORS:
            for x, y in [(0.0, 0.0), (1500.0, 0.0), (0.0, -1500.0), (-800.0, 1200.0), (250.5, 250.5)]:
                with self.subTest(anchor=(lat0, lon0), xy=(x, y)):
                    lat, lon = planner._f2c_xy_to_latlon(x, y, lat0, lon0)
                    x2, y2 = planner._f2c_latlon_to_xy(lat, lon, lat0, lon0)
                    self.assertAlmostEqual(x2, x, places=6)
                    self.assertAlmostEqual(y2, y, places=6)

    def test_distances_match_great_circle_within_field_scale(self):
        """Equirectangular error stays under 0.01 % out to ~1.4 km from the anchor."""
        for lat0, lon0 in ANCHORS:
            for x, y in [(500.0, 0.0), (0.0, 500.0), (500.0, 500.0), (1000.0, -1000.0)]:
                with self.subTest(anchor=(lat0, lon0), xy=(x, y)):
                    lat, lon = planner._f2c_xy_to_latlon(x, y, lat0, lon0)
                    expected = haversine_m(lat0, lon0, lat, lon)
                    self.assertAlmostEqual(math.hypot(x, y), expected, delta=expected * 1e-4)


class FieldCentroidTest(unittest.TestCase):

    def setUp(self):
        silence_planner_log(self)

    def test_rectangle_centroid_is_its_midpoint(self):
        corners = to_ll([(0, 0), (100, 0), (100, 50), (0, 50)])
        x, y = planner.field_centroid_xy(corners)
        self.assertAlmostEqual(x, 50.0, places=6)
        self.assertAlmostEqual(y, 25.0, places=6)

    def test_winding_direction_does_not_matter(self):
        corners = to_ll([(0, 0), (100, 0), (100, 50), (0, 50)])
        x, y = planner.field_centroid_xy(corners)
        x_rev, y_rev = planner.field_centroid_xy([corners[0], *reversed(corners[1:])])
        self.assertAlmostEqual(x, x_rev, places=6)
        self.assertAlmostEqual(y, y_rev, places=6)

    def test_frame_is_anchored_at_first_corner(self):
        """Starting from the opposite corner puts the anchor there: centroid is (-50, -25)."""
        corners = to_ll([(100, 50), (0, 50), (0, 0), (100, 0)])
        x, y = planner.field_centroid_xy(corners)
        self.assertAlmostEqual(x, -50.0, places=2)
        self.assertAlmostEqual(y, -25.0, places=2)

    def test_self_intersecting_boundary_does_not_raise(self):
        bowtie = to_ll([(0, 0), (10, 10), (10, 0), (0, 10)])
        x, y = planner.field_centroid_xy(bowtie)
        self.assertTrue(math.isfinite(x))
        self.assertTrue(math.isfinite(y))


if __name__ == '__main__':
    unittest.main()
