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
from ament_index_python.packages import get_package_share_directory

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
    """ROS entry point for colcon; execution is deferred to NiceGUI startup."""
    pass


def on_startup() -> None:
    """Configures the hardware system and launches the ROS spin thread."""
    # Priority 1: Standard ROS 2 share directory (post-build)
    try:
        pkg_share = get_package_share_directory('devkit_launch')
        config_path = Path(pkg_share) / 'config' / 'feldfreund.py'
    except Exception:
        config_path = Path('/workspace/src/devkit_launch/config/feldfreund.py')

    # Priority 2: Direct workspace source path (for development/hot-reloading)
    if not config_path.exists():
        config_path = Path('/workspace/src/devkit_launch/config/feldfreund.py')

    # Priority 3: Relative path from script (host-side execution fallback)
    if not config_path.exists():
        config_path = Path(__file__).parents[3] / 'devkit_launch/config/feldfreund.py'

    if not config_path.exists():
        # Node cannot safely initialize without a hardware definition
        print(f"FATAL: Configuration file not found at {config_path}")
        os._exit(1)

    print(f"SYSTEM: Loading hardware configuration from {config_path}")
    
    config = config_from_file(str(config_path))
    system = System(config)
    api.Online()
    
    # Background thread handles ROS middleware events without blocking NiceGUI
    threading.Thread(target=ros_main, args=(system,), daemon=True).start()


def ros_main(system: System) -> None:
    """Primary ROS 2 executor loop."""
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

# NiceGUI-specific configuration for Jazzy compatibility
ui_run.APP_IMPORT_STRING = f'{__name__}:app'

ui.run(
    uvicorn_reload_dirs=str(Path(__file__).parent.resolve()),
    title='Agroecology Lab Sowbot UI',
    favicon='assets/favicon.ico',
    dark=True
)
