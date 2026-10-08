import contextlib
import io
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

import yaml

from get_maize_topo import generate, kmeans_1d


class TestMaizeTopologyDefaults(unittest.TestCase):
    def test_generated_maps_use_geometry_rows_and_shared_navigation_defaults(self) -> None:
        """Verify both crop orientations generate geometry row edges and shared map defaults."""
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


class TestMaizeTopologyRowCount(unittest.TestCase):
    def test_single_cluster_centroid_is_the_mean(self) -> None:
        """Verify k=1 returns the mean instead of dividing by zero."""
        self.assertEqual(kmeans_1d([0.0, 1.0, 5.0], 1), [2.0])
        self.assertEqual(kmeans_1d([3.0, 3.0], 1), [3.0])

    def test_non_positive_cluster_count_is_rejected(self) -> None:
        """Verify k below one raises a clear error."""
        for k in (0, -2):
            with self.subTest(k=k), self.assertRaisesRegex(ValueError, 'at least 1'):
                kmeans_1d([0.0, 1.0], k)

    def test_one_row_map_has_one_row_pair_in_both_orientations(self) -> None:
        """Verify --rows 1 writes a map with exactly R1 for NS and EW fields."""
        for orientation in ('NS', 'EW'):
            with self.subTest(orientation=orientation), TemporaryDirectory() as directory:
                csv_path = Path(directory) / 'crops.csv'
                output = Path(directory) / 'map.yaml'
                points = [(0.0, 0), (0.2, 5), (0.1, 10)]
                if orientation == 'EW':
                    points = [(y, x) for x, y in points]
                csv_path.write_text('kind,X,Y\n' + ''.join(
                    f'crop,{x},{y}\n' for x, y in points), encoding='utf-8')

                with contextlib.redirect_stdout(io.StringIO()):
                    generate(csv_path, output, 'field', 1, 1.5, 51.0, -2.0, 10.0)

                doc = yaml.safe_load(output.read_text(encoding='utf-8'))
                names = {entry['node']['name'] for entry in doc['nodes']}
                self.assertEqual(names & {'R1_IN', 'R1_OUT'}, {'R1_IN', 'R1_OUT'})
                self.assertFalse(any(name.startswith('R2') for name in names))
                hub = 'HL_S' if orientation == 'NS' else 'HL_W'
                hub_node = next(e['node'] for e in doc['nodes'] if e['node']['name'] == hub)
                self.assertIn(f'{hub}_R1_IN', {edge['edge_id'] for edge in hub_node['edges']})
                row_pos = next(e['node'] for e in doc['nodes'] if e['node']['name'] == 'R1_IN')
                axis = 'x' if orientation == 'NS' else 'y'
                self.assertAlmostEqual(row_pos['pose']['position'][axis], 0.1, places=3)

    def test_zero_or_negative_rows_exit_cleanly_without_writing(self) -> None:
        """Verify --rows below one exits with a message and leaves no output file."""
        for rows in (0, -1):
            with self.subTest(rows=rows), TemporaryDirectory() as directory:
                csv_path = Path(directory) / 'crops.csv'
                output = Path(directory) / 'map.yaml'
                csv_path.write_text('kind,X,Y\ncrop,0,0\ncrop,0,5\n', encoding='utf-8')
                with self.assertRaises(SystemExit) as raised:
                    generate(csv_path, output, 'field', rows, 1.5, 51.0, -2.0, 10.0)
                self.assertIn('at least 1', str(raised.exception))
                self.assertFalse(output.exists())
