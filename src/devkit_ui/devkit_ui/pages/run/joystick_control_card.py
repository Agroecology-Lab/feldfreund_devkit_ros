from nicegui import ui

from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.run_view_model import RunViewModel


class JoystickControlCard(ui.card):
    def __init__(self, global_vm: GlobalViewModel, run_vm: RunViewModel):
        """
        Initialize the joystick control card with its state and control callbacks.

        Parameters:
            global_vm (GlobalViewModel): Global application state containing the emergency-stop status.
            run_vm (RunViewModel): Run view model containing the joystick state and controls.
        """

        super().__init__()

        self.classes('shrink-0')

        with self:
            ui.label('Joystick').classes('sec-label')

            with ui.row().classes('items-center gap-6'):
                ui.joystick(
                    color='#1a7f37',
                    size=130,
                    on_move=lambda e: run_vm.move_joystick(float(e.y), float(e.x)),
                    on_end=lambda _: run_vm.stop_joystick()
                )

                with ui.column().classes('gap-3 items-center'):
                    # Emergency stop
                    self.estop_btn = ui.button(
                        on_click=lambda _: global_vm.toggle_estop()
                    ).props('outline no-caps').classes('estop-btn')

                    def sync_estop_state(is_active: bool) -> str:
                        self.estop_btn.props(f'color={"negative" if is_active else "primary"}')
                        return 'STOPPED' if is_active else 'E-Stop'

                    self.estop_btn.bind_text_from(
                        global_vm, 'soft_estop_active',
                        backward=sync_estop_state
                    )

                    # Pose label
                    ui.label().bind_text_from(run_vm.joystick, 'pose_lbl').classes(
                        'text-xs font-mono text-[#57606a] text-center max-w-[130px] whitespace-pre-line'
                    )
