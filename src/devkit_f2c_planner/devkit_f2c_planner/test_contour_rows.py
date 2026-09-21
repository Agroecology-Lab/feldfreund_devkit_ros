"""Unit tests for _run_contour_f2c() (shapely offset rows; no fields2cover involved)."""
# pylint: disable=protected-access
import unittest

from shapely.geometry import LineString, Point, Polygon

from devkit_f2c_planner.f2c_test_helpers import planner, silence_planner_log, to_ll, to_xy

FIELD_XY = [(0, 0), (100, 0), (100, 50), (0, 50)]
REFERENCE_XY = [(0, 25), (100, 25)]
TOOL_WIDTH = 4.0  # rows at y = 1, 5, ..., 49: none sits exactly on the field boundary
TOLERANCE_M = 1e-3


def contour_rows(reference_xy: list | None = None, obstacles_xy: list | None = None, **kwargs) -> list:
    """Run _run_contour_f2c() on the fixture field; returns rows in local xy metres."""
    rows_ll = planner._run_contour_f2c(
        to_ll(FIELD_XY), [to_ll(ring) for ring in obstacles_xy or []],
        to_ll(reference_xy or REFERENCE_XY), TOOL_WIDTH, **kwargs)
    return [to_xy(row) for row in rows_ll]


def row_y(row: list) -> float:
    return sum(y for _, y in row) / len(row)


class ContourRowsTest(unittest.TestCase):

    def setUp(self):
        silence_planner_log(self)

    def test_rows_are_spaced_by_the_tool_width_and_span_the_field(self):
        rows = sorted(contour_rows(), key=row_y)
        self.assertEqual(len(rows), 13)
        for index, row in enumerate(rows):
            with self.subTest(row=index):
                self.assertAlmostEqual(row_y(row), 1.0 + TOOL_WIDTH * index, delta=TOLERANCE_M)
                self.assertAlmostEqual(min(x for x, _ in row), 0.0, delta=TOLERANCE_M)
                self.assertAlmostEqual(max(x for x, _ in row), 100.0, delta=TOLERANCE_M)

    def test_every_row_stays_inside_the_field(self):
        field = Polygon(FIELD_XY).buffer(TOLERANCE_M)
        for row in contour_rows():
            self.assertTrue(field.contains(LineString(row)))

    def test_snake_order_alternates_row_direction(self):
        rows = contour_rows(snake_order=True)
        for index, row in enumerate(rows):
            with self.subTest(row=index):
                going_east = row[-1][0] > row[0][0]
                self.assertEqual(going_east, index % 2 == 0)

    def test_without_snake_every_row_keeps_the_reference_direction(self):
        for row in contour_rows(snake_order=False):
            self.assertGreater(row[-1][0], row[0][0])

    def test_headland_insets_the_rows_from_the_boundary(self):
        rows = contour_rows(headland_width_m=2.0)
        self.assertEqual(len(rows), 11)  # y = 1 and y = 49 fall inside the 2 m headland
        for row in rows:
            self.assertGreaterEqual(min(x for x, _ in row), 2.0 - TOLERANCE_M)
            self.assertLessEqual(max(x for x, _ in row), 98.0 + TOLERANCE_M)
            self.assertGreaterEqual(row_y(row), 2.0)
            self.assertLessEqual(row_y(row), 48.0)

    def test_headland_wider_than_the_field_falls_back_to_the_full_boundary(self):
        self.assertEqual(len(contour_rows(headland_width_m=30.0)), len(contour_rows()))

    def test_obstacle_splits_rows_and_no_fragment_enters_it(self):
        obstacle_xy = [(40, 20), (60, 20), (60, 30), (40, 30)]  # rows at y = 21, 25, 29 cross it
        rows = contour_rows(obstacles_xy=[obstacle_xy], snake_order=False)
        self.assertEqual(len(rows), 13 - 3 + 6)
        obstacle = Polygon(obstacle_xy)
        for row in rows:
            self.assertLess(LineString(row).intersection(obstacle).length, TOLERANCE_M)

    def test_max_rows_each_side_limits_the_row_count(self):
        rows = sorted(contour_rows(max_rows_each_side=2), key=row_y)
        self.assertEqual([round(row_y(row)) for row in rows], [17, 21, 25, 29, 33])

    def test_reference_line_with_one_point_returns_no_rows(self):
        self.assertEqual(contour_rows(reference_xy=[(0, 25)]), [])

    def test_reference_line_outside_the_field_returns_no_rows(self):
        self.assertEqual(contour_rows(reference_xy=[(0, 500), (100, 500)]), [])

    def test_curved_reference_gives_rows_at_constant_distance(self):
        """Rows follow the reference: every point of a row is a whole number of tool widths away from it."""
        reference_xy = [(x, 22.5 + 0.002 * (x - 50) ** 2) for x in range(0, 101, 2)]
        reference = LineString(reference_xy)
        rows = contour_rows(reference_xy=reference_xy, snake_order=False)
        self.assertGreater(len(rows), 5)
        for index, row in enumerate(rows):
            with self.subTest(row=index):
                distances = [reference.distance(Point(pt)) for pt in row]
                self.assertLess(max(distances) - min(distances), 2e-3)
                mean_in_widths = sum(distances) / len(distances) / TOOL_WIDTH
                self.assertAlmostEqual(mean_in_widths, round(mean_in_widths), delta=1e-3)


if __name__ == '__main__':
    unittest.main()
