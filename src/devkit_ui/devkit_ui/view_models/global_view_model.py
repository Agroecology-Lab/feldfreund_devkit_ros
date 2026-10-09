from devkit_ui.facades.drive_facade import DriveFacade


class GlobalViewModel:
    def __init__(self, drive_facade: DriveFacade) -> None:
        """Initialize the view model with the soft emergency stop inactive."""
        self.soft_estop_active: bool = False
        self._drive_facade = drive_facade

    def toggle_estop(self) -> None:
        self.soft_estop_active = self._drive_facade.toggle_estop(
            self.soft_estop_active)
