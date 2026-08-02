from collections.abc import Callable

from nicegui import ui

from devkit_ui.stores.run_store import run_store


class TrackCard(ui.card):
    def __init__(self,
                 on_start: Callable[[str, float, int | None, str], None],
                 on_stop: Callable[[], None]):
        super().__init__()

        self.on_start = on_start
        self.on_stop = on_stop

        self.classes('flex-1')

        with self:
            with ui.row().classes('items-baseline gap-2 mb-2'):
                ui.label('Track').classes('font-semibold')
                ui.label('auto-drop every N s').classes('text-xs').style('color:#8c959f')

            with ui.row().classes('items-center gap-2 w-full'):
                self.track_prefix = ui.input(
                    placeholder='Prefix e.g. ROW_A', label='Prefix',
                ).classes('flex-1')
                self.track_interval = ui.number(
                    label='s', value=5, min=2, max=30, step=1, precision=0,
                ).classes('w-16')

            with ui.row().classes('items-center gap-2 w-full mt-1'):
                run_store.track_row_id = ui.number(
                    label='Row ID', placeholder='blank=standard',
                    min=1, step=1, precision=0,
                ).classes('w-28')
                run_store.track_row_role = ui.toggle(
                    {'entry': 'Entry', 'exit': 'Exit'}, value='entry',
                ).props('dense')
                run_store.track_row_hint = ui.label('').classes('text-xs font-mono').style('color:#8c959f')

            with ui.row().classes('items-center gap-2 mt-2'):
                run_store.track_start_btn = ui.button(
                    'Start',
                    on_click=self._handle_start,
                ).props('color=positive no-caps dense')

                run_store.track_stop_btn = ui.button(
                    'Stop',
                    on_click=lambda _: self.on_stop(),
                ).props('color=negative no-caps dense')

                run_store.track_status_lbl = ui.label('').classes(
                    'text-xs font-mono ml-1').style('color:#57606a')

    def _handle_start(self):
        """Extracts input values, handles fallbacks, and triggers on_start callback."""
        prefix = self.track_prefix.value or ''
        interval = float(self.track_interval.value or 5)

        # Read from store values
        row_id = int(run_store.track_row_id.value) if run_store.track_row_id.value is not None else None
        row_role = run_store.track_row_role.value

        self.on_start(prefix, interval, row_id, row_role)
