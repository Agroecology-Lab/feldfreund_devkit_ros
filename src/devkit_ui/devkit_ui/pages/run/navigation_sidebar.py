from collections.abc import Callable, Iterable

from nicegui import ui

from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel


class NavigationSidebar(ui.card):
    def __init__(self,
                 global_store: GlobalViewModel,
                 topo_state: RunViewModel.Topo,
                 on_go: Callable[[], None],
                 on_cancel: Callable[[], None],
                 on_delete: Callable[[], None],
                 on_select: Callable[[str], None]):
        super().__init__()

        self._on_select = on_select

        self.classes('nav-sidebar')

        with self:
            # Current Node
            ui.label('Current node').classes('sec-label')
            ui.label('').classes('text-sm font-mono font-bold').bind_text_from(
                topo_state,
                'current_node',
                backward=lambda current: current or '—'
            )

            # Destination Node
            ui.label('Destination').classes('sec-label mt-3')
            self.selected = ui.label('').classes('text-sm font-mono').style(
                'color:#8c959f'
            )

            def sync_selected(selected: str | None) -> str:
                self.selected.style(
                    f'color:{"#9a6700" if selected else "#8c959f"}'
                )
                return selected or '—'

            self.selected.bind_text_from(
                topo_state, 'selected_node', backward=sync_selected
            )

            # Navigation Status
            ui.label('Status').classes('sec-label mt-3')
            self.nav_status = ui.label('').classes('text-xs font-mono')

            def sync_status(status: str) -> str:
                color = (
                    '#1a7f37' if status == 'arrived' else
                    '#cf222e' if 'fail' in (status or '') else
                    '#57606a'
                )
                self.nav_status.style(f'color:{color}')
                return status

            self.nav_status.bind_text_from(
                topo_state, 'nav_status', backward=sync_status
            )

            ui.separator().classes('my-2')

            # Go button
            ui.button(
                'Go', on_click=on_go, color='positive'
            ).classes('w-full').props('no-caps').bind_enabled_from(
                topo_state,
                'selected_node',
                backward=lambda selected: (
                    bool(selected)
                    and not topo_state.navigating
                    and not global_store.soft_estop_active
                )
            )

            # Cancel button
            ui.button(
                'Cancel', on_click=on_cancel, color='negative'
            ).classes('w-full').props('no-caps flat').bind_enabled_from(
                topo_state,
                'navigating'
            )

            # Delete button
            ui.button(
                'Delete Node', on_click=on_delete, color='negative'
            ).classes('w-full').props('no-caps outline').bind_enabled_from(
                topo_state,
                'selected_node',
                backward=lambda selected: bool(selected)
            )

            # Delete status
            ui.label('Status').classes('sec-label mt-3')
            self.delete_status = ui.label('').classes('text-xs font-mono')

            def sync_delete_status(status: str) -> str:
                color = (
                    '#cf222e' if status.startswith('ERROR') else
                    '#1a7f37' if status else
                    '#57606a'
                )
                self.delete_status.style(f'color:{color};word-break:break-all')
                return status

            self.delete_status.bind_text_from(
                topo_state, 'delete_status', backward=sync_delete_status
            )

            # Nodes list
            ui.label('Nodes').classes('sec-label mt-3')
            with ui.scroll_area().style('flex:1;min-height:0'):
                self.node_col = ui.column().style('gap:1px;width:100%')

    def render_nodes(self, nodes: Iterable, selected_node: str | None) -> None:
        self.node_col.clear()
        with self.node_col:
            for node in sorted(nodes, key=lambda item: item.name):
                is_row = node.meta.get('row_id') is not None
                row_id = node.meta.get('row_id', '')
                row_role = node.meta.get('row_role', '')
                gps_lat = node.meta.get('gps_lat', '')
                gps_lon = node.meta.get('gps_lon', '')
                title = (f'Row {row_id} {row_role} | {gps_lat} {gps_lon}'.strip()
                         if is_row else f'{gps_lat} {gps_lon}'.strip())
                classes = ('node-item'
                           + (' sel' if node.name == selected_node else '')
                           + (' row' if is_row else ''))
                ui.html(
                    f'<div class="{classes}" title="{title}">'
                    f'{node.name}{"  · row" if is_row else ""}</div>'
                ).on('click', lambda _, name=node.name: self._on_select(name))
