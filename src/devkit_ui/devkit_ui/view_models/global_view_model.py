class GlobalViewModel:
    def __init__(self) -> None:
        """Initialize the view model with the soft emergency stop inactive."""
        self.soft_estop_active: bool = False
