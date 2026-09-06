from collections.abc import Callable

from nicegui import ui

from devkit_ui.view_models.run_view_model import RunViewModel


class RowDiscoveryCard(ui.card):
    def __init__(self,
                 state: RunViewModel.Discovery,
                 on_start: Callable[[], None],
                 on_stop: Callable[[], None]):
        """
                 Initialize the row discovery card with its state and lifecycle callbacks.
                 
                 Parameters:
                     state (RunViewModel.Discovery): Discovery state bound to the card controls.
                     on_start (Callable[[], None]): Callback invoked after discovery is confirmed.
                     on_stop (Callable[[], None]): Callback invoked when discovery is disabled.
                 """
        super().__init__()

        self._state = state
        self.on_start = on_start
        self.on_stop = on_stop

        self.classes('flex-1')

        with self:
            ui.label('Row Discovery').classes('font-semibold mb-2')
            ui.html(
                '<div style="font-size:12px;color:#9a6700;'
                'background:#fff8dc;border:1px solid #d4a72c;'
                'border-radius:4px;padding:6px 8px;margin-bottom:8px;">'
                'Discovery mode drives the robot autonomously '
                'through the field with no operator override. '
                'Only enable this with the person-detection '
                'safety system active and confirmed running.</div>'
            )

            with ui.row().classes('items-center gap-2'):
                discovery_checkbox = ui.checkbox('Discovery mode').bind_value_from(
                    self._state, 'active'
                )
                ui.label().classes('text-xs font-mono ml-2').bind_text_from(
                    self._state, 'status'
                )

            async def _on_discovery_change(e, _cb=discovery_checkbox) -> None:
                """Handle changes to the row-discovery activation checkbox.
                
                When enabling discovery, request confirmation that the person-detection safety
                system is active before starting. Cancelled or declined confirmation resets the
                checkbox; disabling discovery stops the process.
                """
                if e.args:  # ticked on
                    with ui.dialog() as d, ui.card():
                        ui.label(
                            'Confirm person-detection safety system'
                        ).classes('font-semibold')
                        ui.label(
                            'Discovery mode will drive the robot with '
                            'no operator override. Confirm the '
                            'person-detection safety system is running '
                            'and active before continuing.'
                        ).classes('text-sm').style('max-width:320px')
                        with ui.row().classes('justify-end gap-2 mt-2'):
                            ui.button(
                                'Cancel',
                                on_click=lambda: d.submit('cancel')
                            ).props('flat no-caps')
                            ui.button(
                                'Confirmed - Start',
                                color='positive',
                                on_click=lambda: d.submit('go')
                            ).props('no-caps')
                    result = await d
                    if result != 'go':
                        _cb.value = False
                        return
                    self.on_start()
                else:
                    self.on_stop()

            discovery_checkbox.on('update:model-value', _on_discovery_change)
