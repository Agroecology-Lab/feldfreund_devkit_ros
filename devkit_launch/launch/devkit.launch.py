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
    ublox_pkg         = get_package_share_directory('ublox_dgnss')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')
    ui_pkg            = get_package_share_directory('devkit_ui')
    fusioncore_pkg    = get_package_share_directory('fusioncore_ros')
    row_follow_pkg    = get_package_share_directory('sowbot_row_follow')

    # ── GPS env vars written by fixusb.py ─────────────────────────────────────
    # mb+r launch files identify hardware by serial string (libusb), not tty.
    rover_serial  = os.getenv('GPS_SERIAL_ROVER',  '')
    rover1_serial = os.getenv('GPS_SERIAL_ROVER1', '')
    gps_type      = os.getenv('GPS_TYPE_ROVER',    'ublox')
    gps1_type     = os.getenv('GPS_TYPE_ROVER1',   'ublox')
    rover_port    = os.getenv('GPS_USB_PATH_ROVER',  os.getenv('GPS_PORT_ROVER',  'virtual'))
    rover1_port   = os.getenv('GPS_USB_PATH_ROVER1', os.getenv('GPS_PORT_ROVER1', 'virtual'))

    mcu_port = os.getenv('MCU_PORT', 'virtual')

    # ── Config paths ──────────────────────────────────────────────────────────
    ntrip_config    = os.path.join(devkit_launch_pkg, 'config', 'ntrip.yaml')
    ntrip_available = os.path.isfile(ntrip_config)

    # fusioncore config: use devkit-specific override if present, else package default
    devkit_fc_config  = os.path.join(devkit_launch_pkg, 'config', 'fusioncore.yaml')
    fusioncore_config = devkit_fc_config if os.path.isfile(devkit_fc_config) \
                        else os.path.join(fusioncore_pkg, 'config', 'fusioncore.yaml')

    # ── Topological map ───────────────────────────────────────────────────────
    # TMAP2_FILE env var overrides the default (use this for real field maps).
    # Falls back to test_simple_tmap2.yaml shipped with topological_navigation.
    # The same path is passed to topo nav as map_path AND exported as TMAP2_FILE
    # so the NiceGUI Navigate tab shows identical nodes to what topo nav routes on.
    topo_nav_share = get_package_share_directory('topological_navigation')
    default_tmap2  = os.path.join(topo_nav_share, 'config', 'test_simple_tmap2.yaml')
    tmap2_file     = os.getenv('TMAP2_FILE', default_tmap2)

    # ── Conditions ────────────────────────────────────────────────────────────
    gps_enabled  = PythonExpression(
        ["'", rover_port,  "' != 'virtual' and '", gps_type,  "' == 'ublox'"]
    )
    gps1_enabled = PythonExpression(
        ["'", rover1_port, "' != 'virtual' and '", gps1_type, "' == 'ublox'"]
    )

    # ── ublox mb+r launch args ─────────────────────────────────────────────────
    # Both mb+r launch files declare device_serial_string, not device/baudrate.
    rover_args = {'device_serial_string': rover_serial  or 'rover'}
    base_args  = {'device_serial_string': rover1_serial or 'base'}

    # ── Optional NTRIP corrections ─────────────────────────────────────────────
    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_client_node',
        name='ntrip_client',
        parameters=[ntrip_config],
        output='screen',
    ) if ntrip_available else None

    # ── Front receiver: mb+r ROVER ─────────────────────────────────────────────
    # Namespace 'rover' set internally by the launch file.
    # Publishes:
    #   /rover/ublox_nav_sat_fix_hp   → relayed to /gnss/fix
    #   /rover/ubx_nav_rel_pos_ned    → consumed by relposned_heading_shim → /gnss/heading
    front_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_rover.launch.py')
            ),
            launch_arguments=rover_args.items(),
        ),
    ]
    if ntrip_node:
        front_actions.append(ntrip_node)

    # ── Rear receiver: mb+r BASE ───────────────────────────────────────────────
    # Sends RTCM corrections to rover over UART2 (physical wire).
    # No ROS topics from base are consumed by fusioncore or topo nav.
    rear_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_base.launch.py')
            ),
            launch_arguments=base_args.items(),
        ),
    ]

    # ── Topic relay: /rover/ublox_nav_sat_fix_hp → /gnss/fix ──────────────────
    # fusioncore hardcodes /gnss/fix (fusion_node.cpp line 355).
    # Only started when real GPS hardware is present.
    gps_fix_relay = Node(
        package='topic_tools',
        executable='relay',
        name='gps_fix_relay',
        arguments=['/rover/ublox_nav_sat_fix_hp', '/gnss/fix'],
        output='screen',
        condition=IfCondition(gps_enabled),
    )

    # ── Topic relay: /odom → /odom/wheels ─────────────────────────────────────
    # devkit_driver publishes wheel odometry on /odom.
    # fusioncore hardcodes /odom/wheels (fusion_node.cpp line 334).
    # Runs in both sim and field: fusioncore degrades gracefully without GPS
    # but needs wheel odometry to function at all.
    # TODO: if devkit_driver can be changed cleanly, rename at source instead.
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_wheels_relay',
        arguments=['/odom', '/odom/wheels'],
        output='screen',
    )

    # ── Heading shim: /rover/ubx_nav_rel_pos_ned → /gnss/heading ──────────────
    # fusioncore subscribes to /gnss/heading as sensor_msgs/Imu (fusion_node.cpp).
    # It extracts yaw from orientation quaternion; orientation_covariance[8] = σ².
    # This shim converts NAV-RELPOSNED (heading + accuracy_deg fields) to that format.
    # Only started when real GPS hardware is present.
    heading_shim = Node(
        package='devkit_driver',
        executable='relposned_heading_shim',
        name='relposned_heading_shim',
        parameters=[{
            'input_topic':    '/rover/ubx_nav_rel_pos_ned',
            'output_topic':   '/gnss/heading',
            'min_baseline_m': 0.5,
        }],
        output='screen',
        condition=IfCondition(gps_enabled),
    )

    # ── fusioncore UKF ─────────────────────────────────────────────────────────
    # Package:    fusioncore_ros
    # Executable: fusioncore_node
    # Not a lifecycle node in practice — launch file starts it directly.
    # Subscribes (hardcoded in source):
    #   /gnss/fix       sensor_msgs/NavSatFix   (from gps_fix_relay)
    #   /gnss/heading   sensor_msgs/Imu          (from heading_shim)
    #   /odom/wheels    nav_msgs/Odometry        (from odom_relay)
    #   /imu/data       sensor_msgs/Imu          (from devkit_driver, if present)
    # Publishes:
    #   /fusion/odom    nav_msgs/Odometry        (GPS-anchored, 100 Hz)
    #   /tf             odom → base_link
    fusioncore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusioncore_pkg, 'launch', 'fusioncore.launch.py')
        ),
        launch_arguments={'fusioncore_config': fusioncore_config}.items(),
        condition=IfCondition(gps_enabled),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),
        SetEnvironmentVariable('TMAP2_FILE', tmap2_file),

        # Front ublox — mb+r ROVER → /rover/*
        GroupAction(
            condition=IfCondition(gps_enabled),
            actions=front_actions,
        ),

        # Rear ublox — mb+r BASE → RTCM over UART2 to rover (no ROS consumers)
        GroupAction(
            condition=IfCondition(gps1_enabled),
            actions=rear_actions,
        ),

        # /rover/ublox_nav_sat_fix_hp → /gnss/fix  (fusioncore hardcoded)
        gps_fix_relay,

        # /odom → /odom/wheels  (fusioncore hardcoded, devkit_driver mismatch)
        odom_relay,

        # NAV-RELPOSNED → sensor_msgs/Imu on /gnss/heading  (fusioncore dual-ant heading)
        heading_shim,

        # fusioncore GPS-anchored UKF → /fusion/odom + odom→base_link TF
        fusioncore_launch,

        # Devkit Driver (motor bridge + wheel odometry → /odom)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(devkit_launch_pkg, 'launch', 'devkit_driver.launch.py')
            ),
            launch_arguments={'port': mcu_port}.items(),
        ),

        # Devkit UI (NiceGUI on :80)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ui_pkg, 'launch', 'ui.launch.py')
            ),
        ),

        # Topological Navigation (LCAS aoc_refactor + fake_nav2_server in sim)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('topological_navigation'),
                    'launch', 'topological_navigation.launch.py'
                )
            ),
            launch_arguments={'map_path': tmap2_file}.items(),
        ),

        # Row following — limbic_row_follow action server on /limbic_row_follow
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(row_follow_pkg, 'launch', 'row_follow.launch.py')
            ),
        ),
    ])
