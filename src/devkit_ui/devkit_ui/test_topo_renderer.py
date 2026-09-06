import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
from xml.etree import ElementTree

from devkit_ui.models import TopoDoc, TopoNode

nicegui = ModuleType('nicegui')
nicegui.ui = SimpleNamespace()

with patch.dict(sys.modules, {'nicegui': nicegui}):
    from devkit_ui.utils.topo_renderer import build_svg


def row_labels(svg: str) -> list[str]:
    root = ElementTree.fromstring(svg)
    return [
        element.text or ''
        for element in root.iter()
        if element.tag.endswith('text') and element.attrib.get('font-size') == '8'
    ]


class TestTopoRendererRowLabels(unittest.TestCase):
    def test_known_row_roles_use_distinct_glyphs(self) -> None:
        doc = TopoDoc(
            name='field',
            nodes=[
                TopoNode(name='ROW_4_IN', x=0.0, y=0.0, meta={'row_id': 4, 'row_role': 'entry'}),
                TopoNode(name='ROW_4_W1', x=1.0, y=0.0, meta={'row_id': 4, 'row_role': 'waypoint'}),
                TopoNode(name='ROW_4_OUT', x=2.0, y=0.0, meta={'row_id': 4, 'row_role': 'exit'}),
            ],
        )

        svg = build_svg(doc, selected=None, current=None)

        self.assertEqual(row_labels(svg), ['I4', 'W4', 'O4'])

    def test_unknown_and_missing_roles_have_safe_fallback_glyphs(self) -> None:
        doc = TopoDoc(
            name='field',
            nodes=[
                TopoNode(name='TURN', x=0.0, y=0.0, meta={'row_id': 8, 'row_role': 'turn'}),
                TopoNode(name='UNKNOWN', x=1.0, y=0.0, meta={'row_id': 9}),
                TopoNode(name='EMPTY', x=2.0, y=0.0, meta={'row_id': 10, 'row_role': ''}),
            ],
        )

        svg = build_svg(doc, selected=None, current=None)

        self.assertEqual(row_labels(svg), ['T8', '?9', '?10'])

    def test_row_label_escapes_non_numeric_identifier(self) -> None:
        doc = TopoDoc(
            name='field',
            nodes=[
                TopoNode(name='ROW_OUT', x=0.0, y=0.0, meta={'row_id': '<7&', 'row_role': 'exit'}),
            ],
        )

        svg = build_svg(doc, selected=None, current=None)

        self.assertIn('O&lt;7&amp;', svg)
        self.assertEqual(row_labels(svg), ['O<7&'])


if __name__ == '__main__':
    unittest.main()
