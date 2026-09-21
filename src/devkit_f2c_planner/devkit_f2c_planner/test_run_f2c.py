"""Unit tests for _run_f2c() orchestration, with fields2cover replaced by FakeF2C.

FakeF2C returns canned swaths, so these tests cover what the planner does around
fields2cover: boundary projection, angle wrapping, headland fallback, snake
ordering, obstacle clipping and projection back to lat/lon. They do not cover
fields2cover's own swath generation.
"""
# pylint: disable=protected-access
import math
import unittest
from unittest import mock

from devkit_f2c_planner.f2c_test_helpers import LAT0, LON0, FakeF2C, planner, silence_planner_log, to_ll, to_xy

FIELD_XY = [(0, 0), (100, 0), (100, 50), (0, 50)]
TOOL_WIDTH = 2.0
ROWS_XY = [[(0.0, y), (100.0, y)] for y in (5.0, 15.0, 25.0, 35.0)]
TOLERANCE_M = 1e-3


class RunF2CTest(unittest.TestCase):

    def setUp(self):
        silence_planner_log(self)

    def run_planner(self, fake: FakeF2C, obstacles_xy: list | None = None, angle_deg: float = 0.0, **kwargs) -> list:
        """Run _run_f2c() against the fake; returns rows in local xy metres."""
        with mock.patch.object(planner, 'f2c', fake):
            rows_ll = planner._run_f2c(
                to_ll(FIELD_XY), [to_ll(ring) for ring in obstacles_xy or []], TOOL_WIDTH, angle_deg, **kwargs)
        return [to_xy(row) for row in rows_ll]

    def assert_rows_almost_equal(self, actual: list, expected: list) -> None:
        self.assertEqual(len(actual), len(expected))
        for actual_row, expected_row in zip(actual, expected, strict=True):
            self.assertEqual(len(actual_row), len(expected_row))
            for (ax, ay), (ex, ey) in zip(actual_row, expected_row, strict=True):
                self.assertAlmostEqual(ax, ex, delta=TOLERANCE_M)
                self.assertAlmostEqual(ay, ey, delta=TOLERANCE_M)

    # ── projection ───────────────────────────────────────────────────

    def test_boundary_is_projected_with_the_first_corner_as_origin(self):
        fake = FakeF2C(ROWS_XY)
        self.run_planner(fake)
        outer_ring = fake.swath_calls[0][2].rings[0]
        self.assert_rows_almost_equal([outer_ring], [FIELD_XY])

    def test_result_is_projected_back_to_latlon_at_the_first_corner(self):
        fake = FakeF2C([[(0.0, 0.0), (100.0, 0.0)]])
        with mock.patch.object(planner, 'f2c', fake):
            (row,) = planner._run_f2c(to_ll(FIELD_XY), [], TOOL_WIDTH, 0.0)
        (lat_start, lon_start), (lat_end, lon_end) = row
        self.assertEqual((lat_start, lon_start), (LAT0, LON0))
        self.assertEqual(lat_end, LAT0)
        self.assertGreater(lon_end, LON0)

    def test_tool_width_is_passed_to_swath_generation(self):
        fake = FakeF2C(ROWS_XY)
        self.run_planner(fake)
        self.assertEqual(fake.swath_calls[0][1], TOOL_WIDTH)

    # ── angle ────────────────────────────────────────────────────────

    def test_angle_is_wrapped_to_a_half_turn_and_passed_in_radians(self):
        for angle_deg, expected_deg in [(0, 0), (10, 10), (190, 10), (-10, 170), (180, 0), (370, 10)]:
            with self.subTest(angle_deg=angle_deg):
                fake = FakeF2C(ROWS_XY)
                self.run_planner(fake, angle_deg=angle_deg)
                self.assertAlmostEqual(fake.swath_calls[0][0], math.radians(expected_deg))

    # ── snake ordering ───────────────────────────────────────────────

    def test_snake_order_reverses_every_second_swath(self):
        rows = self.run_planner(FakeF2C(ROWS_XY), snake_order=True)
        self.assert_rows_almost_equal(rows, [
            [(0.0, 5.0), (100.0, 5.0)],
            [(100.0, 15.0), (0.0, 15.0)],
            [(0.0, 25.0), (100.0, 25.0)],
            [(100.0, 35.0), (0.0, 35.0)],
        ])

    def test_without_snake_order_swaths_keep_the_direction_f2c_gave_them(self):
        rows = self.run_planner(FakeF2C(ROWS_XY), snake_order=False)
        self.assert_rows_almost_equal(rows, ROWS_XY)

    def test_swaths_with_fewer_than_two_points_are_ignored_and_do_not_shift_the_snake_order(self):
        rows = self.run_planner(FakeF2C([[(0.0, 1.0)], *ROWS_XY[:2]]), snake_order=True)
        self.assert_rows_almost_equal(rows, [[(0.0, 5.0), (100.0, 5.0)], [(100.0, 15.0), (0.0, 15.0)]])

    # ── obstacles ────────────────────────────────────────────────────

    def test_obstacle_clips_swaths_even_though_f2c_ignored_it(self):
        obstacle = [(40, 20), (60, 20), (60, 30), (40, 30)]  # crosses the y = 25 swath only
        rows = self.run_planner(FakeF2C(ROWS_XY), [obstacle], snake_order=False)
        self.assert_rows_almost_equal(rows, [
            ROWS_XY[0], ROWS_XY[1], [(0.0, 25.0), (40.0, 25.0)], [(60.0, 25.0), (100.0, 25.0)], ROWS_XY[3],
        ])

    def test_clipped_fragments_follow_the_snake_direction(self):
        obstacle = [(40, 10), (60, 10), (60, 20), (40, 20)]  # crosses the reversed y = 15 swath
        rows = self.run_planner(FakeF2C(ROWS_XY[:2]), [obstacle], snake_order=True)
        self.assert_rows_almost_equal(rows, [
            ROWS_XY[0], [(100.0, 15.0), (60.0, 15.0)], [(40.0, 15.0), (0.0, 15.0)],
        ])

    def test_obstacle_padding_widens_the_gap(self):
        obstacle = [(40, 20), (60, 20), (60, 30), (40, 30)]
        rows = self.run_planner(FakeF2C(ROWS_XY[2:3]), [obstacle], snake_order=False, obstacle_pad_m=5.0)
        self.assert_rows_almost_equal(rows, [[(0.0, 25.0), (35.0, 25.0)], [(65.0, 25.0), (100.0, 25.0)]])

    def test_fragments_shorter_than_half_the_tool_width_are_dropped(self):
        obstacle = [(0.4, 20), (60, 20), (60, 30), (0.4, 30)]  # leaves a 0.4 m stub, below TOOL_WIDTH / 2
        rows = self.run_planner(FakeF2C(ROWS_XY[2:3]), [obstacle], snake_order=False)
        self.assert_rows_almost_equal(rows, [[(60.0, 25.0), (100.0, 25.0)]])

    def test_fragments_longer_than_half_the_tool_width_are_kept(self):
        obstacle = [(1.5, 20), (60, 20), (60, 30), (1.5, 30)]  # leaves a 1.5 m stub, above TOOL_WIDTH / 2
        rows = self.run_planner(FakeF2C(ROWS_XY[2:3]), [obstacle], snake_order=False)
        self.assert_rows_almost_equal(rows, [[(0.0, 25.0), (1.5, 25.0)], [(60.0, 25.0), (100.0, 25.0)]])

    def test_obstacles_are_also_given_to_f2c_as_hole_rings(self):
        fake = FakeF2C(ROWS_XY)
        self.run_planner(fake, [[(40, 20), (60, 20), (60, 30), (40, 30)], [(10, 10), (20, 10), (20, 20)]])
        self.assertEqual(len(fake.swath_calls[0][2].rings), 3)  # boundary + two obstacles

    def test_invalid_obstacle_is_not_given_to_f2c(self):
        fake = FakeF2C(ROWS_XY)
        self.run_planner(fake, [[(0, 0), (5, 0), (10, 0)]])
        self.assertEqual(len(fake.swath_calls[0][2].rings), 1)

    # ── headland ─────────────────────────────────────────────────────

    def test_no_headland_uses_the_full_boundary_without_calling_the_generator(self):
        fake = FakeF2C(ROWS_XY)
        self.run_planner(fake, headland_width_m=0.0)
        self.assertEqual(fake.headland_widths, [])
        self.assertIsNot(fake.swath_calls[0][2], fake.inset_cell)

    def test_headland_swaths_are_generated_on_the_inset_cell(self):
        fake = FakeF2C(ROWS_XY)
        self.run_planner(fake, headland_width_m=3.0)
        self.assertEqual(fake.headland_widths, [3.0])
        self.assertIs(fake.swath_calls[0][2], fake.inset_cell)

    def test_headland_that_leaves_no_cells_falls_back_to_the_full_boundary(self):
        fake = FakeF2C(ROWS_XY, headland_cells=[])
        rows = self.run_planner(fake, headland_width_m=3.0, snake_order=False)
        self.assertEqual(fake.headland_widths, [3.0])
        self.assertEqual(len(fake.swath_calls[0][2].rings), 1)
        self.assertEqual(len(rows), len(ROWS_XY))

    def test_headland_generator_failure_falls_back_to_the_full_boundary(self):
        fake = FakeF2C(ROWS_XY, headland_error=RuntimeError('degenerate field'))
        rows = self.run_planner(fake, headland_width_m=3.0, snake_order=False)
        self.assertEqual(len(fake.swath_calls[0][2].rings), 1)
        self.assertEqual(len(rows), len(ROWS_XY))


if __name__ == '__main__':
    unittest.main()
