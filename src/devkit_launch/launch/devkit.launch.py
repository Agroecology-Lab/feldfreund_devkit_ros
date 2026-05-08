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
    ublox_pkg         = get_package_share_directory('ublox_dgnss')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')
    ui_pkg            = get_package_share_directory('devkit_ui')

    # mb+r launch files identify hardware by serial string, not USB bus path.
    # GPS_PORT_ROVER (tty) is used only to detect whether hardware is present.
    rover_port   = os.getenv('GPS_PORT_ROVER',  'virtual')
    rover_serial = os.getenv('GPS_SERIAL_ROVER', '')
    gps_type     = os.getenv('GPS_TYPE_ROVER',  'ublox')

    rover1_port   = os.getenv('GPS_PORT_ROVER1', 'virtual')
    rover1_serial = os.getenv('GPS_SERIAL_ROVER1', '')
    gps1_type     = os.getenv('GPS_TYPE_ROVER1', 'none')

    mcu_port     = os.getenv('MCU_PORT',    'virtual')
    tmap2_file   = os.getenv('TMAP2_FILE',  '')

    fusioncore_config = os.path.join(devkit_launch_pkg, 'config', 'fusioncore.yaml')

    # Single F9P: front present, rear absent.
    # Uses ublox_single.launch.py which sets LOAD_CONFIG_VIEW=false, avoiding
    # the CFG-VALGET timeout on firmware < HPG 1.32.
    single_gps_enabled = PythonExpression([
        "'", rover_port,  "' != 'virtual' and '", gps_type,  "' == 'ublox' and "
        "'", rover1_port, "' == 'virtual'"
    ])

    # Dual F9P: both present. Uses mb+r rover mode for RELPOSNED heading.
    dual_gps_enabled = PythonExpression([
        "'", rover_port,  "' != 'virtual' and '", gps_type,  "' == 'ublox' and "
        "'", rover1_port, "' != 'virtual' and '", gps1_type, "' == 'ublox'"
    ])

    # Rear F9P only when both are present.
    rear_gps_enabled = PythonExpression([
        "'", rover1_port, "' != 'virtual' and '", gps1_type, "' == 'ublox'"
    ])

    # Serial args for mb+r launches (only used in dual path)
    front_args = {'device_family': 'F9P'}
    if rover_serial:
        front_args['device_serial_string'] = rover_serial

    rear_args = {'device_family': 'F9P'}
    if rover1_serial:
        rear_args['device_serial_string'] = rover1_serial

    topo_args = {'map_file': tmap2_file} if tmap2_file else {}

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # ── Single F9P path ───────────────────────────────────────────────────
        # ublox_single.launch.py: sets LOAD_CONFIG_VIEW=false (no VALGET timeout),
        # launches ublox_dgnss + ublox_nav_sat_fix_hp in the rover namespace.
        # Publishes /rover/ublox_nav_sat_fix_hp for the relay below.
        # RELPOSNED arrives from the F9P (TOML default enables it) but the
        # relposned_heading_shim rejects all messages — correct for single-antenna.
        GroupAction(
            condition=IfCondition(single_gps_enabled),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(devkit_launch_pkg, 'launch', 'ublox_single.launch.py')
                    ),
                ),
            ],
        ),

        # ── Dual F9P path (RTK + heading) ─────────────────────────────────────
        # Base F9P (rear) — start before rover so RTCM is ready on UART2
        GroupAction(
            condition=IfCondition(rear_gps_enabled),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_base.launch.py')
                    ),
                    launch_arguments=rear_args.items(),
                ),
            ],
        ),

        # Rover F9P (front) in moving-base rover mode — produces NavSatFix + RELPOSNED
        GroupAction(
            condition=IfCondition(dual_gps_enabled),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_rover.launch.py')
                    ),
                    launch_arguments=front_args.items(),
                ),
            ],
        ),

        # /rover/ublox_nav_sat_fix_hp → /gnss/fix  (fusioncore GNSS input)
        # Topic is published by both ublox_single and mb+r_rover paths.
        # Relay idles silently when topic absent (sim mode) — always safe to run.
        Node(
            package='topic_tools',
            executable='relay',
            name='navsatfix_relay',
            arguments=['/rover/ublox_nav_sat_fix_hp', '/gnss/fix'],
            output='screen',
        ),

        # /odom → /odom/wheels  (explicit fusioncore odom input)
        # Relay preserves /odom for anything else that needs it.
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_wheels_relay',
            arguments=['/odom', '/odom/wheels'],
            output='screen',
        ),

        # NAV-RELPOSNED → /gnss/heading (sensor_msgs/Imu, ENU yaw quaternion)
        # Only publishes when relPosValid + relPosHeadingValid flags are set.
        # In single-F9P mode: receives RELPOSNED but rejects all (no second antenna).
        # In dual-F9P mode: publishes heading once baseline >= 0.3 m and RTK converges.
        Node(
            package='devkit_driver',
            executable='relposned_heading_shim',
            name='relposned_heading_shim',
            output='screen',
        ),

        # FusionCore UKF — fuses /gnss/fix + /gnss/heading + /odom/wheels
        # Publishes /fusion/odom and odom → base_link TF.
        Node(
            package='fusioncore_ros',
            executable='fusioncore_node',
            name='fusioncore',
            parameters=[fusioncore_config],
            output='screen',
        ),

        # Devkit Driver (Lizard ESP32 bridge — publishes /odom, /battery_state, etc.)
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
            launch_arguments=topo_args.items(),
        ),
    ])
