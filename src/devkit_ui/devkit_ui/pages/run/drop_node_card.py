from collections.abc import Callable

from nicegui import ui

from devkit_ui.constants import NAV_ACTION, ROW_ACTION
from devkit_ui.view_models.run_view_model import RunViewModel


class DropNodeCard(ui.card):
    def __init__(self,
                 state: RunViewModel.DropNode,
                 topo_state: RunViewModel.Topo,
                 on_drop: Callable[[str, int | None, str], None]):
        """
                 Configure the drop-node card with state bindings and a drop action callback.
                 
                 Parameters:
                     state: Drop-node configuration and operation status state.
                     topo_state: Topology state used to display the current node.
                     on_drop: Callback invoked with the node name, row ID, and row role.
                 """
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
                    """
                    Update the row-action hint styling based on whether a row ID is provided.
                    
                    Parameters:
                        row_id (str): The row ID used to determine the hint state.
                    
                    Returns:
                        str: `ROW_ACTION` when a row ID is provided, otherwise `NAV_ACTION`.
                    """
                    has_row = bool(row_id)
                    self.row_hint.style(f'color:{"#0969da" if has_row else "#8c959f"}')
                    return ROW_ACTION if has_row else NAV_ACTION

                self.row_hint.bind_text_from(state, 'row_id', backward=sync_hint)

            with ui.row().classes('items-center gap-2 mt-2'):
                self.current_node_lbl = ui.label('').classes('text-xs font-mono')

                def sync_current_node(current_node: str) -> str:
                    """
                    Update the current-node display and provide its text.
                    
                    Parameters:
                        current_node (str): The current topology node name, or a dash indicating no node.
                    
                    Returns:
                        str: The formatted current-node text.
                    """
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
                """
                Update the status label color based on the status and return the status text.
                
                Parameters:
                    status (str): Operation status text.
                
                Returns:
                    str: The unchanged status text.
                """
                self.status_lbl.style(f'color:{"#cf222e" if status.startswith("ERROR") else "#1a7f37"}')
                return status

            self.status_lbl.bind_text_from(state, 'status', backward=sync_status)
