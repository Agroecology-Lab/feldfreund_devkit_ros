"""Import smoke tests for the driver handler modules.

They guard against the dependency/API breakage that the rosys / feldfreund_devkit
migration can introduce. A ROS environment is required (the modules import rclpy and
the ROS message packages), so these run in the ``ros-tests`` CI job, not the pip-only
``code-checks`` jobs. Importing the modules at collection time is the actual check.
"""

from devkit_driver.qos import SAFETY_QOS

from devkit_driver import modules


def test_qos_profile_is_configured() -> None:
    assert SAFETY_QOS.depth == 1


def test_all_handler_modules_import() -> None:
    handlers = (
        modules.BMSHandler,
        modules.BumperHandler,
        modules.EStopHandler,
        modules.OdomHandler,
        modules.RobotBrainHandler,
        modules.TwistHandler,
    )
    assert all(handler is not None for handler in handlers)
