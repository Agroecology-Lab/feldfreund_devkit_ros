from feldfreund_devkit.config import (
    BluetoothConfiguration,
    BumperConfiguration,
    FeldfreundConfiguration,
    FlashlightConfiguration,
    ImuConfiguration,
    ODriveTracksConfiguration,
    RobotBrainConfiguration,
    Secrets,
)
from rosys.geometry import Rotation


def build_config(secrets: Secrets) -> FeldfreundConfiguration:  # pylint: disable=unused-argument
    # NOTE: `secrets` is required by the `config_from_file` contract but unused here (no cameras / no secret-bearing modules).
    return FeldfreundConfiguration(
        robot_id='Feldfreund',
        bluetooth=BluetoothConfiguration(name='feldfreund DevKit', pin_code=None),
        cameras=None,
        bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),
        flashlight=FlashlightConfiguration(),
        gnss=None,
        implement=None,
        imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.603092, 0.020933, 1.570120),
                             min_gyro_calibration=0.0),
        robot_brain=RobotBrainConfiguration(name='rb57', nand=True),
        wheels=ODriveTracksConfiguration(is_left_reversed=True,
                                         is_right_reversed=False,
                                         left_back_can_address=0x000,
                                         left_front_can_address=0x100,
                                         right_back_can_address=0x200,
                                         right_front_can_address=0x300,
                                         odrive_version=6),
    )
