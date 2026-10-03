import copy
import unittest

from devkit_ui.models import TopoDoc, TopoEdge, TopoNode


class TestTopoDoc(unittest.TestCase):
    def test_switch_counts_edges_across_nodes_and_preserves_edge_identity(self) -> None:
        a = TopoNode(name='A', edges=[
            TopoEdge('row_traversal', 'forward', 'B'),
            TopoEdge('navigate_to_pose', 'headland', 'C'),
        ])
        b = TopoNode(name='B', edges=[TopoEdge('row_traversal', 'reverse', 'A')])
        c = TopoNode(name='C', edges=[TopoEdge('limbic_row_follow', 'vision', 'B')])
        doc = TopoDoc(name='field', nodes=[a, b, c])

        self.assertEqual(doc.set_row_action({'row_traversal', 'limbic_row_follow'},
                                          'limbic_row_follow'), 2)

        self.assertEqual(list(a.edges), [TopoEdge('limbic_row_follow', 'forward', 'B'),
                                         TopoEdge('navigate_to_pose', 'headland', 'C')])
        self.assertEqual(list(b.edges), [TopoEdge('limbic_row_follow', 'reverse', 'A')])
        self.assertEqual(list(c.edges), [TopoEdge('limbic_row_follow', 'vision', 'B')])

    def test_switch_empty_or_unmatched_actions_is_a_noop(self) -> None:
        node = TopoNode(name='A', edges=[TopoEdge('custom', 'A_B', 'B')])
        original = copy.deepcopy(node.to_dict())
        for actions in (set(), {'row_traversal'}, {'custom'}):
            with self.subTest(actions=actions):
                self.assertEqual(node.set_edge_actions(actions, 'custom'), 0)
                self.assertEqual(node.to_dict(), original)
        self.assertEqual(TopoNode(name='empty').set_edge_actions({'custom'}, 'new'), 0)
        self.assertEqual(TopoDoc(name='empty').set_row_action({'row_traversal'}, 'new'), 0)

    def test_renamed_copy_preserves_payload_and_isolates_nested_mutations(self) -> None:
        doc = TopoDoc(
            name='live', metric_map='occupancy', pointset='old',
            meta={'origin': {'latitude': 51.0}},
            actions={'custom': {'options': [1]}}, definitions={'bt': '<root/>'},
            transformation={'translation': {'x': 2.0}},
            nodes=[TopoNode(name='A', x=1.0, y=2.0, pointset='old',
                            edges=[TopoEdge('custom', 'edge', 'B')],
                            meta={'map': 'old', 'pointset': 'old', 'tags': ['crop']}),
                   TopoNode(name='B', x=3.0, y=4.0)],
        )
        original = copy.deepcopy(doc.to_dict())
        expected = copy.deepcopy(original)
        expected['name'] = expected['pointset'] = 'copy'
        for node in expected['nodes']:
            node['node']['pointset'] = 'copy'
            node['meta'].update(map='copy', pointset='copy')

        renamed = doc.renamed('copy')
        self.assertEqual(renamed.to_dict(), expected)
        renamed.actions['custom']['options'].append(2)
        renamed.definitions['bt'] = '<changed/>'
        renamed.transformation['translation']['x'] = 99.0
        renamed.to_dict()['meta']['origin']['latitude'] = 0.0
        renamed.get_node('A').meta['tags'].append('changed')
        renamed.get_node('A').remove_edge('B')
        self.assertEqual(doc.to_dict(), original)

    def test_renamed_empty_document_preserves_occupancy_map(self) -> None:
        doc = TopoDoc(name='live', metric_map='occupancy')
        renamed = doc.renamed('copy').to_dict()
        self.assertEqual((renamed['name'], renamed['pointset']), ('copy', 'copy'))
        self.assertEqual(renamed['metric_map'], 'occupancy')
        self.assertEqual(renamed['nodes'], [])

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


def test_set_row_action_only_touches_row_edges():
    """Verify row-action changes preserve navigation edges and count only changed edges."""
    a = TopoNode(name='a', x=0.0, y=0.0, edges=[
        TopoEdge(action='row_traversal', edge_id='a_b', node='b'),
        TopoEdge(action='navigate_to_pose', edge_id='a_c', node='c')])
    b = TopoNode(name='b', x=1.0, y=0.0)
    c = TopoNode(name='c', x=2.0, y=0.0)
    doc = TopoDoc(name='t', nodes=[a, b, c])
    rows = {'row_traversal', 'limbic_row_follow'}
    assert doc.set_row_action(rows, 'limbic_row_follow') == 1
    acts = {e.node: e.action for e in a.edges}
    assert acts == {'b': 'limbic_row_follow', 'c': 'navigate_to_pose'}
    assert doc.set_row_action(rows, 'limbic_row_follow') == 0
    assert doc.set_row_action(rows, 'row_traversal') == 1


def test_renamed_copy_follows_new_name_and_leaves_original():
    """Verify a renamed copy updates map metadata without changing the original document."""
    doc = TopoDoc(name='live', nodes=[TopoNode(name='a', x=0.0, y=0.0)])
    doc.ensure_meta('live')
    new = doc.renamed('north_field')
    d = new.to_dict()
    assert d['name'] == d['pointset'] == 'north_field'
    assert d['nodes'][0]['meta']['map'] == 'north_field'
    assert d['nodes'][0]['node']['pointset'] == 'north_field'
    assert doc.to_dict()['name'] == 'live'
    assert doc.to_dict()['nodes'][0]['meta']['map'] == 'live'
