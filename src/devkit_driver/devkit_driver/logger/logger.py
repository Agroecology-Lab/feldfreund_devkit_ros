""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""


class Logger:
    """Handle the logger base class."""

    def debug(self, line: str) -> None:
        """Print debug message."""
        raise NotImplementedError()

    def info(self, line: str) -> None:
        """Print info message."""
        raise NotImplementedError()

    def warn(self, line: str) -> None:
        """Print warn message."""
        raise NotImplementedError()

    def error(self, line: str) -> None:
        """Print error message."""
        raise NotImplementedError()
