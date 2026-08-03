from nicegui import ui

from devkit_ui.models import TopoDoc
from devkit_ui.stores.run_store import run_store
from devkit_ui.utils.topo_renderer import build_robot_svg, build_svg


class NodeMapCard(ui.card):
    def __init__(self, topo_doc: TopoDoc):
        super().__init__()

        self._topo_doc = topo_doc

        self.classes('flex-1')

        run_store.map_svg = build_svg(self._topo_doc, None, None)
        run_store.robot_svg = build_robot_svg(self._topo_doc.nodes, None)

        with self:
            ui.label('Node Map').classes('sec-label')

            # Node map + robot marker overlay (same viewBox, so they
            # align). Overlay is pointer-events:none so clicks reach
            # the nodes; it updates on movement without rebuilding
            # the clickable node DOM.
            with ui.element('div').classes('relative w-full'):
                ui.html().classes('w-full').bind_content_from(run_store, 'map_svg')

                ui.html().classes('absolute top-0 left-0 w-full') \
                    .style('pointer-events:none') \
                    .bind_content_from(run_store, 'robot_svg')
