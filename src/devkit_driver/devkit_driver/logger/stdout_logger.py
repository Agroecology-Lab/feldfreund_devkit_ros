""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

import logging

from .logger import Logger


class StdOutLogger(Logger):
    """Implements a logger that outputs to stdout."""

    def __init__(self) -> None:
        super().__init__()
        self._logger = logging.getLogger()

    def set_level(self, logging_level: int):
        """Set logging level."""
        self._logger.setLevel(logging_level)

    def debug(self, line: str) -> None:
        """Print debug message."""
        self._logger.debug(line)

    def info(self, line: str) -> None:
        """Print info message."""
        self._logger.info(line)

    def warn(self, line: str) -> None:
        """Print warn message."""
        self._logger.warning(line)

    def error(self, line: str) -> None:
        """Print error message."""
        self._logger.error(line)
