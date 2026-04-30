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
from launch_ros.actions import Node


def generate_launch_description():
    ublox_pkg = get_package_share_directory('ublox_dgnss')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')
    ui_pkg = get_package_share_directory('devkit_ui')

    # mb+r launch files identify hardware by serial string, not USB bus path.
    # GPS_PORT_ROVER (tty path) is used only to detect whether hardware is present.
    rover_port   = os.getenv('GPS_PORT_ROVER', 'virtual')
    rover_serial = os.getenv('GPS_SERIAL_ROVER', '')
    gps_type     = os.getenv('GPS_TYPE_ROVER', 'ublox')

    rover1_port   = os.getenv('GPS_PORT_ROVER1', 'virtual')
    rover1_serial = os.getenv('GPS_SERIAL_ROVER1', '')
    gps1_type     = os.getenv('GPS_TYPE_ROVER1', 'ublox')

    mcu_port = os.getenv('MCU_PORT', 'virtual')
    tmap2_file = os.getenv('TMAP2_FILE', '')
    fusioncore_config = os.path.join(devkit_launch_pkg, 'config', 'fusioncore.yaml')

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

    # Rover (front antenna) — moving-base rover mode
    # Publishes /rover/ublox_nav_sat_fix_hp and /rover/ubx_nav_rel_pos_ned
    front_args = {'device_family': 'F9P'}
    if rover_serial:
        front_args['device_serial_string'] = rover_serial

    # Base (rear antenna) — moving-base base mode
    # Sends RTCM to rover via UART2 (physical cable) — no ROS topics consumed
    rear_args = {'device_family': 'F9P'}
    if rover1_serial:
        rear_args['device_serial_string'] = rover1_serial

    front_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_rover.launch.py')
            ),
            launch_arguments=front_args.items(),
        ),
    ]

    rear_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_base.launch.py')
            ),
            launch_arguments=rear_args.items(),
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # Rear u-blox — moving-base base (start before rover so RTCM is ready)
        GroupAction(
            condition=IfCondition(gps1_enabled),
            actions=rear_actions,
        ),

        # Front u-blox — moving-base rover
        GroupAction(
            condition=IfCondition(gps_enabled),
            actions=front_actions,
        ),

        # /rover/ublox_nav_sat_fix_hp → /gnss/fix  (fusioncore GNSS input)
        # /odom                       → /odom/wheels (fusioncore odom input;
        #   relay not remap so /odom remains live for anything else)
        # Both always launched — relay nodes are harmless when topic absent
        Node(
            package='topic_tools',
            executable='relay',
            name='navsatfix_relay',
            arguments=['/rover/ublox_nav_sat_fix_hp', '/gnss/fix'],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_wheels_relay',
            arguments=['/odom', '/odom/wheels'],
            output='screen',
        ),

        # NAV-RELPOSNED → /gnss/heading (compass_msgs/Compass, ENU radians)
        # Only publishes when relPosValid + relPosHeadingValid flags set.
        # Hardware-gated: shim silently idles when /rover/ubx_nav_rel_pos_ned absent.
        Node(
            package='devkit_driver',
            executable='relposned_heading_shim',
            name='relposned_heading_shim',
            output='screen',
        ),

        # FusionCore UKF — fuses /gnss/fix + /gnss/heading + /odom/wheels
        # Publishes /fusion/odom and odom → base_link TF
        Node(
            package='fusioncore_ros',
            executable='fusioncore_node',
            name='fusioncore',
            parameters=[fusioncore_config],
            output='screen',
        ),

        # Devkit Driver (The Bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(devkit_launch_pkg, 'launch', 'devkit_driver.launch.py')
            ),
            launch_arguments={'port': mcu_port}.items(),
        ),

        # Devkit UI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ui_pkg, 'launch', 'ui.launch.py')
            ),
        ),

        # Topological Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('topological_navigation'),
                    'launch', 'topological_navigation.launch.py'
                )
            ),
            launch_arguments=({'map_file': tmap2_file}.items()
                              if tmap2_file else {}.items()),
        ),
    ])
