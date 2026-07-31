from nicegui import ui

from devkit_ui.models import TopoDoc
from devkit_ui.stores.run_store import run_store
from devkit_ui.utils.topo_renderer import _build_robot_svg, _build_svg


class NodeMapCard(ui.card):
    def __init__(self, topo_doc: TopoDoc):
        super().__init__()

        self._topo_doc = topo_doc

        self.classes('flex-1')

        with self:
            ui.label('Node Map').classes('sec-label')

            # Node map + robot marker overlay (same viewBox, so they
            # align). Overlay is pointer-events:none so clicks reach
            # the nodes; it updates on movement without rebuilding
            # the clickable node DOM.
            with ui.element('div').classes('relative w-full'):
                run_store.map_html = ui.html(
                    _build_svg(self._topo_doc, None, None)
                ).classes('w-full')

                run_store.robot_html = ui.html(
                    _build_robot_svg(self._topo_doc.nodes, None)
                ).classes('absolute top-0 left-0 w-full').style('pointer-events:none')
