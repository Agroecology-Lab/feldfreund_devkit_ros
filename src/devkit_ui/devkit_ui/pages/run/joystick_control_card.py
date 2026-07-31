from collections.abc import Callable

from nicegui import events, ui

from devkit_ui.logging import app_logger
from devkit_ui.stores.global_store import global_store
from devkit_ui.stores.run_store import run_store


class JoystickControlCard(ui.card):
    def __init__(self,
                 on_move: Callable[[float, float], None],
                 on_stop: Callable[[], None],
                 on_estop: Callable[[bool], None]):
        super().__init__()

        self.on_move = on_move
        self.on_stop = on_stop
        self.on_estop = on_estop

        self.classes('shrink-0')

        with self:
            ui.label('Joystick').classes('sec-label')

            with ui.row().classes('items-center gap-6'):
                ui.joystick(
                    color='#1a7f37',
                    size=130,
                    on_move=self._handle_joystick_move,
                    on_end=self._handle_joystick_end,
                )

                with ui.column().classes('gap-3 items-center'):
                    self.estop_btn = ui.button(
                        on_click=lambda _: self.on_estop()
                    ).props('outline no-caps').classes('estop-btn')

                    def sync_estop_state(is_active: bool) -> str:
                        # Side-effect: update the color prop
                        self.estop_btn.props(f'color={"negative" if is_active else "primary"}')
                        # Return the text that bind_text_from is expecting
                        return 'STOPPED' if is_active else 'E-Stop'

                    self.estop_btn.bind_text_from(
                        global_store, 'soft_estop_active',
                        backward=sync_estop_state
                    )

                    # pose_lbl
                    ui.label().bind_text_from(run_store, 'pose_lbl').classes(
                        'text-xs font-mono text-[#57606a] text-center max-w-[130px] whitespace-pre-line'
                    )

    def _log_event(self, e: events.JoystickEventArguments):
        if not app_logger:
            return

        x_value = e.x if e.x is not None else 0.0
        y_value = e.y if e.y is not None else 0.0

        app_logger.info(
            f'[joystick debug] end event raw e.x={e.x!r} e.y={e.y!r} '
            f'(as float: x={float(x_value):+.3f} y={float(y_value):+.3f})'
        )

    def _handle_joystick_move(self, e: events.JoystickEventArguments):
        self._log_event(e)
        self.on_move(float(e.y), float(e.x))

    def _handle_joystick_end(self, e: events.JoystickEventArguments):
        self._log_event(e)
        self.on_stop()
