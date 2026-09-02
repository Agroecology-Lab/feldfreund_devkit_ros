from collections.abc import Callable

from nicegui import ui

from devkit_ui.constants import NAV_ACTION, ROW_ACTION
from devkit_ui.stores.run_store import RunStore


class DropNodeCard(ui.card):
    def __init__(self,
                 state: RunStore.DropNode,
                 topo_state: RunStore.Topo,
                 on_drop: Callable[[str, int | None, str], None]):
        super().__init__()

        self.classes('flex-1')

        with self:
            with ui.row().classes('items-baseline gap-2 mb-2'):
                # Label
                ui.label('Drop Node').classes('font-semibold')
                ui.label('pins at current pose').classes('text-xs').style('color:#8c959f')

            with ui.row().classes('items-center gap-2 w-full'):
                # Name
                ui.input(
                    placeholder='e.g. ROW_D_IN', label='Name',
                ).classes('flex-1').bind_value(state, 'name')

            with ui.row().classes('items-center gap-2 w-full mt-1'):
                # Row id
                ui.number(
                    label='Row ID', placeholder='blank=standard',
                    min=1, step=1, precision=0,
                ).classes('w-28').bind_value(
                    state, 'row_id',
                    forward=lambda val: int(val) if val not in (None, '') else None
                )

                # Toggle role
                ui.toggle(
                    {'entry': 'Entry', 'exit': 'Exit'}
                ).props('dense').bind_value(state, 'row_role')

                # Hint label
                self.row_hint = ui.label('').classes('text-xs font-mono')

                def sync_hint(row_id: str) -> str:
                    has_row = bool(row_id)
                    self.row_hint.style(f'color:{"#0969da" if has_row else "#8c959f"}')
                    return ROW_ACTION if has_row else NAV_ACTION

                self.row_hint.bind_text_from(state, 'row_id', backward=sync_hint)

            with ui.row().classes('items-center gap-2 mt-2'):
                self.current_node_lbl = ui.label('').classes('text-xs font-mono')

                def sync_current_node(current_node: str) -> str:
                    has_current = bool(current_node and current_node != '—')
                    self.current_node_lbl.style(
                        f'color:{"#1a7f37" if has_current else "#8c959f"}'
                    )
                    return f'→ {current_node}' if has_current else 'no current node'

                self.current_node_lbl.bind_text_from(
                    topo_state, 'current_node', backward=sync_current_node
                )

                ui.button(
                    'Drop',
                    on_click=lambda: on_drop(
                        state.name,
                        state.row_id,
                        state.row_role,
                    ),
                ).classes('ml-auto').props('color=positive no-caps dense')

            self.status_lbl = ui.label('').classes('text-xs font-mono mt-1')

            def sync_status(status: str) -> str:
                self.status_lbl.style(f'color:{"#cf222e" if status.startswith("ERROR") else "#1a7f37"}')
                return status

            self.status_lbl.bind_text_from(state, 'status', backward=sync_status)
