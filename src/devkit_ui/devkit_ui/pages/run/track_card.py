from collections.abc import Callable

from nicegui import ui

from devkit_ui.constants import NAV_ACTION
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
                # Label
                ui.label('Track').classes('font-semibold')
                ui.label('auto-drop every N s').classes('text-xs').style('color:#8c959f')

            with ui.row().classes('items-center gap-2 w-full'):
                # Prefix
                ui.input(
                    placeholder='Prefix e.g. ROW_A', label='Prefix',
                ).classes('flex-1').bind_value(run_store, 'track_prefix')

                # Interval
                ui.number(
                    label='s', value=5, min=2, max=30, step=1, precision=0,
                ).classes('w-16').bind_value(run_store, 'track_interval')

            with ui.row().classes('items-center gap-2 w-full mt-1'):
                # Row id
                ui.number(
                    label='Row ID', placeholder='blank=standard',
                    min=1, step=1, precision=0,
                ).classes('w-28').bind_value(
                    run_store, 'track_row_id',
                    backward=lambda val: int(val) if val not in (None, '') else None
                )

                # Toggle role
                ui.toggle(
                    {'entry': 'Entry', 'exit': 'Exit'}
                ).props('dense').bind_value(run_store, 'track_row_role').bind_visibility_from(
                    run_store, 'track_row_id',
                    backward=lambda val: not bool(val)
                )

                # Hint label
                self.track_hint = ui.label().classes('text-xs font-mono')

                def sync_hint(row_id: int | None) -> str:
                    has_row = bool(row_id)
                    self.track_hint.style(f'color:{"#0969da" if has_row else "#8c959f"}')
                    return 'entry→middle→exit auto' if has_row else NAV_ACTION

                self.track_hint.bind_text_from(run_store, 'track_row_id', backward=sync_hint)

            with ui.row().classes('items-center gap-2 mt-2'):
                # Start button
                start_btn = ui.button(
                    'Start',
                    on_click=lambda: self.on_start(
                        run_store.track_prefix,
                        float(run_store.track_interval or 5),
                        run_store.track_row_id,
                        run_store.track_row_role,
                    ),
                ).props('color=positive no-caps dense')
                start_btn.bind_enabled_from(
                    run_store, 'track_running', backward=lambda running: not running
                )

                # Stop button
                stop_btn = ui.button(
                    'Stop',
                    on_click=lambda _: self.on_stop(),
                ).props('color=negative no-caps dense')
                stop_btn.bind_enabled_from(run_store, 'track_running')

                # Status label
                self.track_status_lbl = ui.label().classes('text-xs font-mono ml-1')

                def sync_status(status: str) -> str:
                    running = run_store.track_running
                    color = (
                        '#cf222e' if status.startswith('ERROR') else
                        '#1a7f37' if running else
                        '#57606a'
                    )
                    self.track_status_lbl.style(f'color:{color}')
                    return status

                self.track_status_lbl.bind_text_from(run_store, 'track_status', backward=sync_status)
