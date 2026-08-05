from nicegui import ui

from devkit_ui.stores.run_store import RunStore


class NodeMapCard(ui.card):
    def __init__(self, state: RunStore.NodeMap):
        super().__init__()

        self._state = state

        self.classes('flex-1')

        with self:
            ui.label('Node Map').classes('sec-label')

            # Node map + robot marker overlay (same viewBox, so they
            # align). Overlay is pointer-events:none so clicks reach
            # the nodes; it updates on movement without rebuilding
            # the clickable node DOM.
            with ui.element('div').classes('relative w-full'):
                ui.html().classes('w-full').bind_content_from(self._state, 'map_svg')

                ui.html().classes('absolute top-0 left-0 w-full') \
                    .style('pointer-events:none') \
                    .bind_content_from(self._state, 'robot_svg')
