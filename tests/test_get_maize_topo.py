import contextlib
import io
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

import yaml

from get_maize_topo import generate


class TestMaizeTopologyDefaults(unittest.TestCase):
    def test_generated_maps_use_geometry_rows_and_shared_navigation_defaults(self) -> None:
        # NOTE: get_maize_topo adds the ROS package source directory to sys.path on import.
        # pylint: disable-next=import-outside-toplevel
        from devkit_ui.topo_defaults import default_actions, default_definitions

        for orientation in ('NS', 'EW'):
            with self.subTest(orientation=orientation), TemporaryDirectory() as directory:
                csv_path = Path(directory) / 'crops.csv'
                output = Path(directory) / 'map.yaml'
                points = [(x, y) for x in (0, 2) for y in (0, 5, 10)]
                if orientation == 'EW':
                    points = [(y, x) for x, y in points]
                csv_path.write_text('kind,X,Y\n' + ''.join(
                    f'crop,{x},{y}\n' for x, y in points), encoding='utf-8')

                with contextlib.redirect_stdout(io.StringIO()):
                    generate(csv_path, output, 'field', 2, 1.5, 51.0, -2.0, 10.0)

                doc = yaml.safe_load(output.read_text(encoding='utf-8'))
                self.assertEqual(doc['actions'], default_actions())
                self.assertEqual(doc['definitions'], default_definitions())
                for definition in doc['definitions'].values():
                    self.assertEqual(ElementTree.fromstring(definition).tag, 'root')
                nodes = {entry['node']['name']: entry['node'] for entry in doc['nodes']}
                row_edges = []
                navigation_edges = []
                for name, node in nodes.items():
                    for edge in node['edges']:
                        if name.startswith('R') and name.endswith('_IN') and edge['node'].endswith('_OUT'):
                            row_edges.append(edge)
                        else:
                            navigation_edges.append(edge)
                self.assertEqual({edge['edge_id'] for edge in row_edges},
                                 {'R1_IN_R1_OUT', 'R2_IN_R2_OUT'})
                self.assertEqual({edge['action'] for edge in row_edges}, {'row_traversal'})
                self.assertTrue(navigation_edges)
                self.assertEqual({edge['action'] for edge in navigation_edges}, {'navigate_to_pose'})
