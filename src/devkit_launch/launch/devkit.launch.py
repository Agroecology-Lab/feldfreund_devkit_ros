import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ublox_pkg = get_package_share_directory('ublox_dgnss_node')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')

    # Front antenna (primary position source)
    rover_port   = os.getenv('GPS_PORT_ROVER',   'virtual')
    rover_serial = os.getenv('GPS_SERIAL_ROVER',  '')
    gps_type     = os.getenv('GPS_TYPE_ROVER',    'ublox')

    # Rear antenna (second receiver for heading baseline)
    rover1_port   = os.getenv('GPS_PORT_ROVER1',  'virtual')
    rover1_serial = os.getenv('GPS_SERIAL_ROVER1', '')
    gps1_type     = os.getenv('GPS_TYPE_ROVER1',   'ublox')

    mcu_port = os.getenv('MCU_PORT', 'virtual')

    # NTRIP config — optional, corrections only flow if the file exists
    ntrip_config = os.path.join(devkit_launch_pkg, 'config', 'ntrip.yaml')
    ntrip_available = os.path.isfile(ntrip_config)

    # Only launch each receiver if its port is physical and type is ublox
    gps_enabled = PythonExpression(
        ["'", rover_port, "' != 'virtual' and '", gps_type, "' == 'ublox'"]
    )
    gps1_enabled = PythonExpression(
        ["'", rover1_port, "' != 'virtual' and '", gps1_type, "' == 'ublox'"]
    )

    # Build launch_arguments for each ublox instance.
    # DEVICE_SERIAL_STRING pins the driver to a specific physical module so
    # two instances do not race to claim the same device.
    front_args = {'device': rover_port, 'baudrate': '460800'}
    if rover_serial:
        front_args['DEVICE_SERIAL_STRING'] = rover_serial

    rear_args = {'device': rover1_port, 'baudrate': '460800'}
    if rover1_serial:
        rear_args['DEVICE_SERIAL_STRING'] = rover1_serial

    # ntrip_client nodes — one per receiver namespace so each ublox instance
    # picks up corrections on its own <namespace>/ntrip_client/rtcm topic.
    # Both connect to the same caster; RTCM corrections are not receiver-specific.
    ntrip_front = Node(
        package='ntrip_client',
        executable='ntrip_client_node',
        name='ntrip_client',
        parameters=[ntrip_config],
        output='screen',
    ) if ntrip_available else None

    ntrip_rear = Node(
        package='ntrip_client',
        executable='ntrip_client_node',
        name='ntrip_client',
        parameters=[ntrip_config],
        output='screen',
    ) if ntrip_available else None

    front_actions = [
        PushRosNamespace('ublox_front'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_pkg, 'launch', 'ublox_rover_hpposllh.launch.py')
            ),
            launch_arguments=front_args.items(),
        ),
    ]
    if ntrip_front:
        front_actions.append(ntrip_front)

    rear_actions = [
        PushRosNamespace('ublox_rear'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_pkg, 'launch', 'ublox_rover_hpposllh.launch.py')
            ),
            launch_arguments=rear_args.items(),
        ),
    ]
    if ntrip_rear:
        rear_actions.append(ntrip_rear)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # Front ublox receiver — namespace: ublox_front
        GroupAction(
            condition=IfCondition(gps_enabled),
            actions=front_actions,
        ),

        # Rear ublox receiver — namespace: ublox_rear
        GroupAction(
            condition=IfCondition(gps1_enabled),
            actions=rear_actions,
        ),

        # Devkit Driver (The Bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(devkit_launch_pkg, 'launch', 'devkit_driver.launch.py')
            ),
            launch_arguments={'port': mcu_port}.items(),
        ),
    ])
