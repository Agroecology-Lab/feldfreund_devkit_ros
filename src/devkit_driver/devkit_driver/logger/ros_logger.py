""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
    Modified by Zauberzeug GmbH
"""

from rclpy.impl.rcutils_logger import RcutilsLogger

from .logger import Logger


class ROSLogger(Logger):
    """Handle the ros logging functionalities."""

    def __init__(self, ros_logger: RcutilsLogger) -> None:
        """Initialize the ros logger class."""
        self._logger = ros_logger

    def debug(self, line: str) -> None:
        """Print debug message."""
        self._logger.debug(line)

    def info(self, line: str) -> None:
        """Print info message."""
        self._logger.info(line)

    def warn(self, line: str) -> None:
        """Print warn message."""
        self._logger.warn(line)

    def error(self, line: str) -> None:
        """Print error message."""
        self._logger.error(line)
