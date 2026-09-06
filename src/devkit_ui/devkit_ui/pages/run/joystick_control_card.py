from collections.abc import Callable

from nicegui import ui

from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel


class JoystickControlCard(ui.card):
    def __init__(self,
                 global_store: GlobalViewModel,
                 state: RunViewModel.Joystick,
                 on_move: Callable[[float, float], None],
                 on_stop: Callable[[], None],
                 on_estop: Callable[[], None]):
        """
                 Initialize the joystick control card with its state and control callbacks.

                 Parameters:
                     global_store (GlobalViewModel): Global application state containing the emergency-stop status.
                     state (RunViewModel.Joystick): Joystick state containing the displayed pose.
                     on_move (Callable[[float, float], None]): Callback invoked with the joystick's y and x coordinates.
                     on_stop (Callable[[], None]): Callback invoked when joystick movement ends.
                     on_estop (Callable[[], None]): Callback invoked when the emergency-stop button is pressed.
                 """
        super().__init__()

        self._global_store = global_store
        self._state = state
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
                    on_move=lambda e: self.on_move(float(e.y), float(e.x)),
                    on_end=lambda _: self.on_stop()
                )

                with ui.column().classes('gap-3 items-center'):
                    # Emergency stop
                    self.estop_btn = ui.button(
                        on_click=lambda _: self.on_estop()
                    ).props('outline no-caps').classes('estop-btn')

                    def sync_estop_state(is_active: bool) -> str:
                        self.estop_btn.props(f'color={"negative" if is_active else "primary"}')
                        return 'STOPPED' if is_active else 'E-Stop'

                    self.estop_btn.bind_text_from(
                        self._global_store, 'soft_estop_active',
                        backward=sync_estop_state
                    )

                    # Pose label
                    ui.label().bind_text_from(self._state, 'pose_lbl').classes(
                        'text-xs font-mono text-[#57606a] text-center max-w-[130px] whitespace-pre-line'
                    )
