#!/usr/bin/env python3

import threading
from pathlib import Path
import os

import rclpy
from feldfreund_devkit import FeldfreundHardware, FeldfreundSimulation, System, api
from feldfreund_devkit.config import config_from_file
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from devkit_driver.modules import (BMSHandler, BumperHandler, EStopHandler, 
                                 OdomHandler, RobotBrainHandler, TwistHandler)


class DevkitDriver(Node):
    """Devkit node handler with defensive attribute checks for simulation support."""

    def __init__(self, system: System):
        super().__init__('devkit_driver_node')
        self.system = system
        
        # NOTE: Support both physical hardware and simulation (GHOST mode)
        assert isinstance(self.system.feldfreund, (FeldfreundHardware, FeldfreundSimulation))
        
        # Defensive initialization: Only load handlers if attributes exist.
        # This prevents the AttributeError: 'FeldfreundSimulation' object has no attribute 'robot_brain'
        if hasattr(self.system.feldfreund, 'robot_brain'):
            self._robot_brain_handler = RobotBrainHandler(self, self.system.feldfreund.robot_brain)
        else:
            self.get_logger().info('RobotBrain not detected (Simulation Mode); skipping handler.')

        # Odometry is standard across both modes
        self._odom_handler = OdomHandler(self, self.system.odometer)
        
        if hasattr(self.system.feldfreund, 'bms'):
            self._bms_handler = BMSHandler(self, self.system.feldfreund.bms)
            
        if getattr(self.system.feldfreund, 'bumper', None) is not None:
            self._bumper_handler = BumperHandler(self, self.system.feldfreund.bumper, self.system.feldfreund.estop)
            
        if hasattr(self.system.feldfreund, 'wheels'):
            self._twist_handler = TwistHandler(self, self.system.feldfreund.wheels)
            
        if hasattr(self.system.feldfreund, 'estop'):
            self._estop_handler = EStopHandler(self, self.system.feldfreund.estop)


def main() -> None:
    # NOTE: This function is the ROS entry point in setup.py, but remains empty
    # to allow NiceGUI to handle the main execution loop and auto-reloading.
    pass


def on_startup() -> None:
    # Resolve configuration path dynamically for container vs host environments
    config_path = Path('/workspace/src/devkit_launch/config/feldfreund.py')
    if not config_path.exists():
        # Fallback to local host development path
        config_path = Path(__file__).parents[3] / 'devkit_launch/config/feldfreund.py'

    config = config_from_file(str(config_path))
    system = System(config)
    api.Online()
    # Daemon thread ensures the background ROS thread exits when the main process stops
    threading.Thread(target=ros_main, args=(system,), daemon=True).start()


def ros_main(system: System) -> None:
    rclpy.init()
    devkit_driver = DevkitDriver(system)
    try:
        rclpy.spin(devkit_driver)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


app.on_startup(on_startup)

# Critical for Jazzy: Map the application import string for NiceGUI auto-reload
ui_run.APP_IMPORT_STRING = f'{__name__}:app'

# Professional UI configuration: Removed emoji, set title for Agroecology Lab standards
ui.run(
    uvicorn_reload_dirs=str(Path(__file__).parent.resolve()),
    title='Agroecology Lab Sowbot UI',
    favicon='assets/favicon.ico'
)
