#!/usr/bin/env python3

import os
import threading
from pathlib import Path

import rclpy
import rclpy.parameter
import rosys
from feldfreund_devkit import FeldfreundHardware, FeldfreundSimulation, System, api
from feldfreund_devkit.config import Secrets, config_from_file
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from devkit_driver.modules import BMSHandler, BumperHandler, EStopHandler, OdomHandler, RobotBrainHandler, TwistHandler


class DevkitDriver(Node):
    """Devkit node handler."""

    def __init__(self, system: System):
        super().__init__('devkit_driver_node')
        self.system = system

        if isinstance(self.system.feldfreund, FeldfreundHardware):
            self.get_logger().info('Running in hardware mode')
            self._robot_brain_handler = RobotBrainHandler(self, self.system.feldfreund.robot_brain)
        elif isinstance(self.system.feldfreund, FeldfreundSimulation):
            self.get_logger().info('Running in simulation mode')
            self._clock_publisher = self.create_publisher(Clock, '/clock', 10)
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
                                                           rclpy.parameter.Parameter.Type.BOOL,
                                                           True)])
            # Start clock publishing using rosys repeater for better integration
            rosys.on_repeat(self._publish_clock, 0.01)
        else:
            raise TypeError(f'Unknown feldfreund type: {type(self.system.feldfreund)}')

        # All other handlers work with both hardware and simulation
        self._odom_handler = OdomHandler(self, self.system.odometer)
        self._bms_handler = BMSHandler(self, self.system.feldfreund.bms)
        if self.system.feldfreund.bumper is not None:
            self._bumper_handler = BumperHandler(self, self.system.feldfreund.bumper, self.system.feldfreund.estop)
        self._twist_handler = TwistHandler(self, self.system.feldfreund.wheels)
        self._estop_handler = EStopHandler(self, self.system.feldfreund.estop)

    def _publish_clock(self) -> None:
        """Publish RoSys simulation time to ROS2 /clock topic."""
        current_time = rosys.time()

        msg = Clock()
        msg.clock.sec = int(current_time)
        msg.clock.nanosec = int((current_time - int(current_time)) * 1e9)

        self._clock_publisher.publish(msg)


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def on_startup() -> None:
    # Check if simulation mode is requested via environment variable
    simulation_mode = os.environ.get('FELDFREUND_SIMULATION', 'false').lower() in ('true', '1', 'yes')
    if simulation_mode:
        rosys.enter_simulation()

    secrets = Secrets()
    config = config_from_file('/workspace/src/devkit_launch/config/feldfreund.py', secrets=secrets)
    system = System(config, secrets=secrets)
    api.Online()
    threading.Thread(target=ros_main, args=(system,)).start()


def ros_main(system: System) -> None:
    rclpy.init()
    devkit_driver = DevkitDriver(system)
    try:
        rclpy.spin(devkit_driver)
    except ExternalShutdownException:
        pass


app.on_startup(on_startup)
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
