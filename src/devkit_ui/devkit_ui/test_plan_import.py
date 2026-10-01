# pylint: disable=missing-docstring
import json
import unittest

from devkit_ui.plan_import import PlanError, parse_plan


def _feat(coords_lonlat, row, frag=0, role='row'):
    return {'type': 'Feature', 'properties': {'role': role, 'row': row, 'frag': frag},
            'geometry': {'type': 'LineString', 'coordinates': coords_lonlat}}


def _plan(features, **props):
    p = {'version': 1, 'origin': [51.45, -2.6], 'generator': 'test'}
    p.update(props)
    return json.dumps({'type': 'FeatureCollection', 'properties': p, 'features': features})


ROW0 = [[-2.6, 51.45], [-2.599, 51.45]]
ROW1 = [[-2.599, 51.4501], [-2.6, 51.4501]]


class ParsePlanTest(unittest.TestCase):
    def test_swaps_lonlat_to_latlon(self):
        plan = parse_plan(_plan([_feat(ROW0, 0)]))
        self.assertEqual(plan.swaths[0][0], (51.45, -2.6))
        self.assertEqual(plan.origin_ll, (51.45, -2.6))

    def test_order_preserved_and_non_rows_ignored(self):
        plan = parse_plan(_plan([_feat(ROW0, 0, role='field'), _feat(ROW0, 0), _feat(ROW1, 1)]))
        self.assertEqual(len(plan.swaths), 2)
        self.assertEqual(plan.swaths[1][0], (51.4501, -2.599))

    def test_fragments_of_one_row_are_not_linked(self):
        plan = parse_plan(_plan([_feat(ROW0, 0, 0), _feat(ROW0, 0, 1), _feat(ROW1, 1)]))
        self.assertEqual(plan.break_after, {0})

    def test_rejects_bad_input(self):
        bad = [
            'not json',
            json.dumps({'type': 'Feature'}),
            _plan([_feat(ROW0, 0)], version=2),
            _plan([_feat(ROW0, 0)], origin=[200, 0]),
            _plan([]),
            _plan([_feat([[-2.6, 51.45]], 0)]),
            _plan([_feat([[-2.6, 91.0], [-2.5, 51.0]], 0)]),
            _plan([_feat([[-2.6, float('nan')], [-2.5, 51.0]], 0)]),
            _plan([_feat(ROW0, -1)]),
            _plan([_feat(ROW0, True)]),
        ]
        for text in bad:
            with self.subTest(text=text[:60]), self.assertRaises(PlanError):
                parse_plan(text)

    def test_distance_guard(self):
        text = _plan([_feat(ROW0, 0)])
        parse_plan(text, robot_ll=(51.4501, -2.5995))
        with self.assertRaises(PlanError):
            parse_plan(text, robot_ll=(28.6, 77.2))


if __name__ == '__main__':
    unittest.main()
