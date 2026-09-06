# F2CSaveHarness is built dynamically (see load_f2c_save_harness below), and
# make_node supplies the attributes normally initialized by the ROS node.
# pylint: disable=attribute-defined-outside-init,exec-used,protected-access
import ast
import math
import re
import unittest
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from devkit_ui.models import NodeID, TopoDoc, TopoEdge, TopoNode, TopoPose, TopoProperties, Vector2

NAV_ACTION = 'nav_to_pose'
ROW_ACTION = 'row_follow'


def latlon_to_xy(lat: float, lon: float, anchor_lat: float, anchor_lon: float) -> tuple[float, float]:
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
    method = next(
        node for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name == 'save_f2c_rows_to_topo'
    )
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
    module = ast.Module(body=[method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), source_path, 'exec'), namespace)
    return type('F2CSaveHarness', (), {'save_f2c_rows_to_topo': namespace['save_f2c_rows_to_topo']})


F2CSaveHarness = load_f2c_save_harness()


def make_node(swaths, *, contour_used=False):
    node = F2CSaveHarness()
    node._f2c_swaths = swaths
    node._f2c_contour_used = contour_used
    node._f2c_origin_ll = (51.0, -2.0)
    node._is_sim = True
    node.latest_odom = None
    node.latest_gps = SimpleNamespace(
        latitude=51.0,
        longitude=-2.0,
        status=SimpleNamespace(status=0),
    )
    node._topo_doc = TopoDoc(name='field')
    node._run_vm = SimpleNamespace(
        topo=SimpleNamespace(current_node=None, selected_node=None),
    )
    node.f2c_save_status = ''
    node.get_logger = Mock(return_value=Mock())

    def persist(modify, status_owner, status_attr, success_message):
        modify(node._topo_doc)
        setattr(status_owner, status_attr, success_message)

    node._persist_and_reload = Mock(side_effect=persist)
    return node


class TestF2CRowMetadata(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
