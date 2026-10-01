"""Tests for the pure-shapely backend, plus an F2C parity check (skipped without fields2cover)."""
# pylint: disable=invalid-name,protected-access
import math
import unittest

from devkit_f2c_planner.f2c_test_helpers import planner, silence_planner_log, to_ll, to_xy

RECT = [(0, 0), (100, 0), (100, 60), (0, 60)]


def _run(backend, **kw):
    """Plan rows for the default rectangle with the chosen backend and option overrides."""
    args = dict(corners_ll=to_ll(RECT), obstacle_rings=[], tool_width=5.0, angle_deg=0.0,
                backend=backend)
    args.update(kw)
    return planner._run_f2c(**args)


class ShapelyBackendTest(unittest.TestCase):
    def setUp(self):
        """Silence planner diagnostics for each test and restore logging afterward."""
        silence_planner_log(self)

    def test_row_count_and_spacing(self):
        """Verify row count, half-width boundary offset, and tool-width spacing."""
        rows = _run('shapely', snake_order=False)
        self.assertEqual(len(rows), 12)
        ys = [to_xy(r)[0][1] for r in rows]
        self.assertAlmostEqual(ys[0], 2.5, places=2)
        for a, b in zip(ys, ys[1:]):
            self.assertAlmostEqual(b - a, 5.0, places=2)

    def test_rows_span_field(self):
        """Verify horizontal swaths span the full rectangular field width."""
        for r in _run('shapely'):
            xy = to_xy(r)
            self.assertAlmostEqual(abs(xy[-1][0] - xy[0][0]), 100.0, places=2)

    def test_snake_alternates_direction(self):
        """Verify snake ordering reverses the travel direction on every other row."""
        rows = _run('shapely')
        dirs = [to_xy(r)[-1][0] - to_xy(r)[0][0] for r in rows]
        self.assertTrue(all(d * dirs[0] * (-1) ** i > 0 for i, d in enumerate(dirs)))

    def test_rotated_angle(self):
        """Verify a 90-degree swath angle produces vertical rows with the expected count."""
        rows = _run('shapely', angle_deg=90.0, snake_order=False)
        self.assertEqual(len(rows), 20)
        xy = to_xy(rows[0])
        self.assertAlmostEqual(xy[0][0], xy[-1][0], places=2)

    def test_headland_shrinks_rows(self):
        """Verify a headland inset shortens rows at both ends."""
        rows = _run('shapely', headland_width_m=10.0, snake_order=False)
        self.assertAlmostEqual(abs(to_xy(rows[0])[-1][0] - to_xy(rows[0])[0][0]), 80.0, places=2)

    def test_obstacle_splits_rows(self):
        """Verify an interior obstacle splits intersecting rows into extra swaths."""
        obs = to_ll([(40, 10), (60, 10), (60, 50), (40, 50)])
        rows = _run('shapely', obstacle_rings=[obs], snake_order=False)
        self.assertGreater(len(rows), 12)

    def test_concave_field_gives_fragments(self):
        """Verify a concave field produces multiple fragments for some rows."""
        u = [(0, 0), (100, 0), (100, 60), (70, 60), (70, 20), (30, 20), (30, 60), (0, 60)]
        rows = planner._run_f2c(to_ll(u), [], 5.0, 0.0, snake_order=False, backend='shapely')
        self.assertGreater(len(rows), 12)

    def test_unknown_backend(self):
        """Verify an unsupported backend name raises ValueError."""
        with self.assertRaises(ValueError):
            _run('nope')


@unittest.skipUnless(hasattr(planner.f2c, 'SG_BruteForce'), 'real fields2cover not installed')
class ParityTest(unittest.TestCase):
    """Run inside the Docker image. Compares swath geometry, ignoring row order/direction."""

    def setUp(self):
        """Silence planner diagnostics for each test and restore logging afterward."""
        silence_planner_log(self)

    def _norm(self, rows):
        """Return sorted endpoint tuples rounded to decimetres, ignoring row direction."""
        out = []
        for r in rows:
            xy = to_xy(r)
            a, b = sorted([xy[0], xy[-1]])
            out.append((round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1)))
        return sorted(out)

    def test_parity(self):
        """Compare backend swath endpoints across field shapes, angles, and headland widths."""
        shapes = {
            'rect': RECT,
            'tri': [(0, 0), (120, 0), (40, 80)],
            'concave': [(0, 0), (100, 0), (100, 60), (70, 60), (70, 20), (30, 20), (30, 60), (0, 60)],
        }
        for name, shape in shapes.items():
            for angle in (0, 30, 90, 137):
                for hl in (0.0, 8.0):
                    with self.subTest(shape=name, angle=angle, headland=hl):
                        kw = dict(corners_ll=to_ll(shape), obstacle_rings=[], tool_width=5.0,
                                  angle_deg=angle, headland_width_m=hl, snake_order=False)
                        self.assertEqual(self._norm(planner._run_f2c(backend='f2c', **kw)),
                                         self._norm(planner._run_f2c(backend='shapely', **kw)))


if __name__ == '__main__':
    unittest.main()


class TagRowsTest(unittest.TestCase):
    def setUp(self):
        """Silence planner diagnostics for each test and restore logging afterward."""
        silence_planner_log(self)

    def test_obstacle_fragments_share_row(self):
        """Verify obstacle fragments share row IDs and remain in row order."""
        obs = to_ll([(40, 10), (60, 10), (60, 50), (40, 50)])
        corners = to_ll(RECT)
        rows = planner._run_f2c(corners, [obs], 5.0, 0.0, snake_order=False, backend='shapely')
        tags = planner.tag_rows(rows, corners, 0.0)
        self.assertEqual(len(tags), len(rows))
        self.assertEqual(max(r for r, _ in tags) + 1, 12)
        self.assertTrue(any(f == 1 for _, f in tags))
        self.assertEqual([r for r, _ in tags], sorted(r for r, _ in tags))

    def test_rotated_rows_tagged(self):
        """Verify rotated snake rows receive consecutive, distinct row IDs."""
        corners = to_ll(RECT)
        rows = planner._run_f2c(corners, [], 5.0, 90.0, snake_order=True, backend='shapely')
        self.assertEqual(sorted({r for r, _ in planner.tag_rows(rows, corners, 90.0)}),
                         list(range(20)))
