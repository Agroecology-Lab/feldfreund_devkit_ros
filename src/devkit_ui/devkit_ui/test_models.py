import unittest

from devkit_ui.models import TopoDoc, TopoEdge, TopoNode


class TestTopoDoc(unittest.TestCase):
    def test_init_with_list_of_dict_nodes(self) -> None:
        doc = TopoDoc(
            name='test-topo',
            nodes=[
                {'name': 'A', 'x': 1.0, 'y': 2.0, 'edges': []},
                {'name': 'B', 'x': 3.0, 'y': 4.0, 'edges': ['A']},
            ],
        )

        self.assertTrue(doc.has_node('A'))
        self.assertTrue(doc.has_node('B'))
        self.assertIsInstance(doc.get_node('A'), TopoNode)
        self.assertEqual([e.node for e in doc.get_node('B').edges], ['A'])

    def test_init_with_list_of_toponodes(self) -> None:
        node_a = TopoNode(name='A', x=0.0, y=0.0)
        node_b = TopoNode(name='B', x=1.0, y=1.0, edges=['A'])

        doc = TopoDoc(name='test-topo', nodes=[node_a, node_b])

        self.assertTrue(doc.has_node('A'))
        self.assertTrue(doc.has_node('B'))
        self.assertEqual([e.node for e in doc.get_node('B').edges], ['A'])

    def test_add_node_constructs_reverse_edges(self) -> None:
        doc = TopoDoc(name='test-topo', nodes=[TopoNode(name='A', x=0.0, y=0.0)])
        node_b = TopoNode(
            name='B',
            x=1.0,
            y=1.0,
            edges=[TopoEdge(action='move_base', edge_id='B_A', node='A')],
        )

        doc.add_node(node_b)

        self.assertTrue(doc.has_node('B'))
        self.assertIn(
            TopoEdge(action='move_base', edge_id='B_A', node='B'),
            list(doc.get_node('A').edges),
        )

    def test_remove_node_removes_related_edges(self) -> None:
        node_a = TopoNode(name='A', x=0.0, y=0.0, edges=['B'])
        node_b = TopoNode(name='B', x=1.0, y=1.0, edges=['A', 'C'])
        node_c = TopoNode(name='C', x=2.0, y=2.0, edges=['B', 'D'])
        node_d = TopoNode(name='D', x=3.0, y=3.0, edges=['C'])
        doc = TopoDoc(name='test-topo', nodes=[node_a, node_b, node_c, node_d])

        doc.remove_node('B')

        self.assertFalse(doc.has_node('B'))
        self.assertEqual(list(doc.get_node('A').edges), [])
        self.assertEqual([e.node for e in doc.get_node('C').edges], ['D'])
        self.assertEqual([e.node for e in doc.get_node('D').edges], ['C'])

if __name__ == '__main__':
    unittest.main()
