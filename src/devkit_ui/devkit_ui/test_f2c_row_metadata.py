# F2CSaveHarness is built dynamically (see load_f2c_save_harness below), and
# make_node supplies the attributes normally initialized by the ROS node.
# pylint: disable=attribute-defined-outside-init,exec-used,no-member,protected-access
import ast
import math
import re
import unittest
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

from devkit_ui.models import NodeID, TopoDoc, TopoEdge, TopoNode, TopoPose, TopoProperties, Vector2

NAV_ACTION = 'nav_to_pose'
ROW_ACTION = 'row_follow'


def latlon_to_xy(
    lat: float, lon: float, anchor_lat: float, anchor_lon: float,
) -> tuple[float, float]:
    return lat - anchor_lat, lon - anchor_lon


def xy_to_latlon(x: float, y: float, anchor_lat: float, anchor_lon: float) -> tuple[float, float]:
    return anchor_lat + x, anchor_lon + y


def resample_row_xy(_points, _anchor_lat, _anchor_lon, _interval):
    return [(1.0, 1.5)]


def load_f2c_save_harness():
    """Load only the F2C save method, avoiding ui_node's ROS and web-server imports."""
    source_path = Path(__file__).with_name('ui_node.py')
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'NiceGuiNode'
    )
    methods = [
        node for node in node_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name in ('save_f2c_rows_to_topo', 'repair_row_connectivity')
    ]
    helpers = [node for node in tree.body
               if isinstance(node, ast.FunctionDef) and node.name == '_headland_neighbour_pairs']
    namespace = {
        'NodeID': NodeID,
        'TopoDoc': TopoDoc,
        'TopoEdge': TopoEdge,
        'TopoNode': TopoNode,
        'TopoPose': TopoPose,
        'TopoProperties': TopoProperties,
        'Vector2': Vector2,
        'NAV_ACTION': NAV_ACTION,
        'ROW_ACTION': ROW_ACTION,
        '_CONTOUR_WAYPOINT_INTERVAL_M': 1.0,
        '_f2c_latlon_to_xy': latlon_to_xy,
        '_f2c_xy_to_latlon': xy_to_latlon,
        '_resample_row_xy': resample_row_xy,
        'UTC': UTC,
        'datetime': datetime,
        'math': math,
        'pairwise': pairwise,
        're': re,
    }
    module = ast.Module(body=[*helpers, *methods], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), source_path, 'exec'), namespace)
    return type('F2CSaveHarness', (), {method.name: namespace[method.name] for method in methods})


F2CSaveHarness = load_f2c_save_harness()


def make_node(swaths, *, contour_used=False):
    """Build a simulated F2C save harness with supplied swaths and in-memory persistence."""
    node = F2CSaveHarness()
    node._f2c_swaths = swaths
    node._f2c_contour_used = contour_used
    node._f2c_origin_ll = (51.0, -2.0)
    node._f2c_break_after = set()
    node._is_sim = True
    node._row_action = ROW_ACTION
    node.latest_odom = None
    node.latest_gps = SimpleNamespace(
        latitude=51.0,
        longitude=-2.0,
        status=SimpleNamespace(status=0),
    )
    node._topo_doc = TopoDoc(name='field')
    node._topo_vm = SimpleNamespace(current_node=None, selected_node=None)
    node.f2c_save_status = ''
    node.get_logger = Mock(return_value=Mock())

    def persist(modify, status_owner, status_attr, success_message):
        modify(node._topo_doc)
        setattr(status_owner, status_attr, success_message)

    node._persist_and_reload = Mock(side_effect=persist)
    return node


class TestF2CRowMetadata(unittest.TestCase):
    def test_repair_preserves_existing_modes_and_is_idempotent_after_mode_change(self) -> None:
        """Verify repair preserves existing actions and skips saving fully connected rows."""
        node = make_node([])
        node._row_action = 'limbic_row_follow'
        for name, role, y in (('IN', 'entry', 0), ('WP', 'waypoint', 5), ('OUT', 'exit', 10)):
            node._topo_doc.insert_node(TopoNode(
                name=name, x=0, y=y, meta={'row_id': 1, 'row_role': role}))
        existing = TopoEdge('row_traversal', 'authored', 'WP')
        node._topo_doc.get_node('IN').add_edge(existing)

        node.repair_row_connectivity()

        self.assertEqual(list(node._topo_doc.get_node('IN').edges), [existing])
        self.assertEqual([edge.action for edge in node._topo_doc.get_node('WP').edges],
                         ['limbic_row_follow'])
        node._persist_and_reload.assert_called_once()
        node._persist_and_reload.reset_mock()
        node._row_action = 'row_traversal'

        node.repair_row_connectivity()

        self.assertEqual(list(node._topo_doc.get_node('IN').edges), [existing])
        self.assertEqual([edge.action for edge in node._topo_doc.get_node('WP').edges],
                         ['limbic_row_follow'])
        self.assertIn('already wired', node.f2c_save_status)
        node._persist_and_reload.assert_not_called()

    def test_selected_mode_applies_to_straight_and_contour_chains_only(self) -> None:
        """Verify both swath types use the selected row mode and navigation for headlands."""
        for action in ('row_traversal', 'limbic_row_follow'):
            for contour in (False, True):
                with self.subTest(action=action, contour=contour):
                    node = make_node([
                        [(51.0, -2.0), (51.0005, -2.0), (51.001, -2.0)],
                        [(51.001, -1.999), (51.0005, -1.999), (51.0, -1.999)],
                    ], contour_used=contour)
                    node._row_action = action
                    node.save_f2c_rows_to_topo('crop', row_id_start=1)
                    row_edges = []
                    headland_edges = []
                    for source in node._topo_doc.nodes:
                        for edge in source.edges:
                            target = node._topo_doc.get_node(edge.node)
                            if source.meta['row_id'] == target.meta['row_id']:
                                row_edges.append(edge)
                            else:
                                headland_edges.append(edge)
                    self.assertEqual(len(row_edges), 4 if contour else 2)
                    self.assertEqual({edge.action for edge in row_edges}, {action})
                    self.assertTrue(headland_edges)
                    self.assertEqual({edge.action for edge in headland_edges}, {NAV_ACTION})

    def test_repair_uses_selected_mode_for_each_waypoint_and_navigation_for_headlands(self):
        """Verify repair applies row actions along waypoint chains and navigation between rows."""
        for action in ('row_traversal', 'limbic_row_follow'):
            with self.subTest(action=action):
                node = make_node([])
                node._row_action = action
                for name, row_id, role, x, y in (
                    ('R1_IN', 1, 'entry', 0, 0), ('R1_W1', 1, 'waypoint', 0, 5),
                    ('R1_OUT', 1, 'exit', 0, 10), ('R2_IN', 2, 'entry', 2, 0),
                    ('R2_OUT', 2, 'exit', 2, 10),
                ):
                    node._topo_doc.insert_node(TopoNode(
                        name=name, x=x, y=y, meta={'row_id': row_id, 'row_role': role}))

                node.repair_row_connectivity()

                actual = {(source.name, edge.node): edge.action
                          for source in node._topo_doc.nodes for edge in source.edges}
                self.assertEqual(actual, {
                    ('R1_IN', 'R1_W1'): action, ('R1_W1', 'R1_OUT'): action,
                    ('R2_IN', 'R2_OUT'): action,
                    ('R1_IN', 'R2_IN'): NAV_ACTION, ('R2_IN', 'R1_IN'): NAV_ACTION,
                    ('R1_OUT', 'R2_OUT'): NAV_ACTION, ('R2_OUT', 'R1_OUT'): NAV_ACTION,
                })
                node._persist_and_reload.assert_called_once()

    def assert_row_metadata(self, node: TopoNode, row_id: int, row_role: str) -> None:
        self.assertEqual(node.meta['row_id'], row_id)
        self.assertEqual(node.meta['row_role'], row_role)
        properties = node.to_dict()['node']['properties']
        self.assertEqual(properties['row_id'], row_id)
        self.assertEqual(properties['row_role'], row_role)

    def test_entry_and_exit_metadata_is_saved_for_each_row(self) -> None:
        node = make_node([
            [(51.0, -2.0), (51.001, -1.999)],
            [(51.002, -2.0), (51.003, -1.999)],
        ])

        node.save_f2c_rows_to_topo('crop', row_id_start=7)

        expected = {
            'CROP_R7_IN': (7, 'entry'),
            'CROP_R7_OUT': (7, 'exit'),
            'CROP_R8_IN': (8, 'entry'),
            'CROP_R8_OUT': (8, 'exit'),
        }
        self.assertEqual({saved.name for saved in node._topo_doc.nodes}, set(expected))
        for name, (row_id, row_role) in expected.items():
            with self.subTest(name=name):
                self.assert_row_metadata(node._topo_doc.get_node(name), row_id, row_role)

    def test_contour_waypoint_metadata_matches_entry_and_exit_metadata(self) -> None:
        node = make_node(
            [[(51.0, -2.0), (51.0005, -1.9995), (51.001, -1.999)]],
            contour_used=True,
        )

        node.save_f2c_rows_to_topo('curve', row_id_start=3)

        expected = {
            'CURVE_R3_IN': 'entry',
            'CURVE_R3_W1': 'waypoint',
            'CURVE_R3_OUT': 'exit',
        }
        self.assertEqual({saved.name for saved in node._topo_doc.nodes}, set(expected))
        for name, row_role in expected.items():
            with self.subTest(name=name):
                self.assert_row_metadata(node._topo_doc.get_node(name), 3, row_role)


class TestDirectContourPlanning(unittest.IsolatedAsyncioTestCase):
    async def test_fragment_breaks_reach_topo_and_reset_for_straight_plans(self):
        swaths = [
            [(51.0, -2.0), (51.001, -2.0)],
            [(51.002, -2.0), (51.003, -2.0)],
            [(51.003, -1.999), (51.0, -1.999)],
        ]
        node = make_node([])
        node._obstacle_mgr = Mock()
        node._obstacle_mgr.rings_ll.return_value = []

        def plan_contours(*_args, break_after):
            break_after.add(0)
            return swaths

        source_path = Path(__file__).with_name('ui_node.py')
        tree = ast.parse(source_path.read_text(encoding='utf-8'))
        methods = [method for method in ast.walk(tree)
                   if isinstance(method, ast.FunctionDef | ast.AsyncFunctionDef)
                   and method.name in ('_plan_contour_rows', 'do_plan')]
        namespace = {
            'self': node, 'corners_ll': [(51.0, -2.0), (51.003, -2.0), (51.0, -1.999)],
            'swath_layers': [], 'mission_map': Mock(),
            'plan_btn': Mock(), 'save_btn': Mock(), 'f2c_status': Mock(),
            'ng_run': SimpleNamespace(
                io_bound=AsyncMock(side_effect=lambda fn, *a, **kw: fn(*a, **kw))),
            '_run_contour_f2c': plan_contours, '_run_f2c': Mock(return_value=swaths),
            'load_recon_points': Mock(return_value=([], [], [])),
            'build_elevation_grid': Mock(return_value=([], (0, 0), 0)),
            'field_centroid_xy': Mock(return_value=(0, 0)),
            'select_reference_contour_latlon': Mock(return_value=swaths[0]),
            'np': Mock(),
        }
        values = {'f2c_width': 4, 'f2c_angle': 0, 'f2c_row_id_start': 7,
                  'f2c_headland': 0, 'f2c_snake': True, 'f2c_contour': True,
                  'obstacle_pad': 0, 'f2c_recon_path': 'test.csv', 'f2c_dem_res': 1}
        namespace.update({name: SimpleNamespace(value=value) for name, value in values.items()})
        module = ast.Module(body=methods, type_ignores=[])
        exec(compile(ast.fix_missing_locations(module), source_path, 'exec'), namespace)

        await namespace['do_plan']()
        self.assertEqual(node._f2c_break_after, {0})
        self.assertTrue(node._f2c_contour_used)
        node.save_f2c_rows_to_topo('curve', row_id_start=7)
        nodes = node._topo_doc
        self.assertNotIn('CURVE_R8_IN', [e.node for e in nodes.get_node('CURVE_R7_OUT').edges])
        self.assertNotIn('CURVE_R7_OUT', [e.node for e in nodes.get_node('CURVE_R8_IN').edges])
        self.assertIn('CURVE_R9_IN', [e.node for e in nodes.get_node('CURVE_R8_OUT').edges])

        namespace['select_reference_contour_latlon'].return_value = None
        await namespace['do_plan']()
        self.assertEqual(node._f2c_break_after, set())
        self.assertFalse(node._f2c_contour_used)

        node._f2c_break_after = {0}
        namespace['f2c_contour'].value = False
        await namespace['do_plan']()
        self.assertEqual(node._f2c_break_after, set())


if __name__ == '__main__':
    unittest.main()
