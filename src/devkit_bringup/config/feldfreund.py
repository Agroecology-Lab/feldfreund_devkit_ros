from feldfreund_devkit.config import (
    BluetoothConfiguration,
    BumperConfiguration,
    FeldfreundConfiguration,
    FlashlightConfiguration,
    ImuConfiguration,
    RobotBrainConfiguration,
    TracksConfiguration,
)
from rosys.geometry import Rotation

config = FeldfreundConfiguration(
    robot_id='Feldfreund',
    bluetooth=BluetoothConfiguration(name='feldfreund DevKit', pin_code=None),
    camera=None,
    bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),
    circle_sight_positions=None,
    flashlight=FlashlightConfiguration(),
    gnss=None,
    implement=None,
    imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.603092, 0.020933, 1.570120), min_gyro_calibration=0.0),
    robot_brain=RobotBrainConfiguration(name='rb57', flash_params=['orin', 'v05', 'nand']),
    wheels=TracksConfiguration(is_left_reversed=True,
                               is_right_reversed=False,
                               left_back_can_address=0x000,
                               left_front_can_address=0x100,
                               right_back_can_address=0x200,
                               right_front_can_address=0x300,
                               odrive_version=6),
)
