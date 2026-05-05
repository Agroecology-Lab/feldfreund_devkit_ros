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
    # NOTE: F9P OEM modules often ship with iSerial=0 (no serial programmed).
    #       In that case rover_serial is '' and we fall back to ublox_single.launch.py
    #       which identifies the device by USB bus path (GPS_USB_PATH_ROVER) instead.
    rover_serial  = os.getenv('GPS_SERIAL_ROVER',  '')
    rover1_serial = os.getenv('GPS_SERIAL_ROVER1', '')
    gps_type      = os.getenv('GPS_TYPE_ROVER',    'ublox')
    gps1_type     = os.getenv('GPS_TYPE_ROVER1',   'ublox')
    rover_port    = os.getenv('GPS_USB_PATH_ROVER',  os.getenv('GPS_PORT_ROVER',  'virtual'))
    rover1_port   = os.getenv('GPS_USB_PATH_ROVER1', os.getenv('GPS_PORT_ROVER1', 'virtual'))

    mcu_port = os.getenv('MCU_PORT', 'virtual')

    # Single-antenna fallback: no serial + no second receiver = one F9P with iSerial=0.
    # ublox_mb+r_rover.launch.py requires device_serial_string — useless when iSerial=0.
    # ublox_single.launch.py (devkit-owned) uses GPS_USB_PATH_ROVER (bus path) directly.
    single_antenna = (rover_serial == '' and rover1_port == 'virtual')

    # ── Config paths ──────────────────────────────────────────────────────────
    ntrip_config    = os.path.join(devkit_launch_pkg, 'config', 'ntrip.yaml')
    ntrip_available = os.path.isfile(ntrip_config)

    # fusioncore config: use devkit-specific override if present, else package default
    devkit_fc_config  = os.path.join(devkit_launch_pkg, 'config', 'fusioncore.yaml')
    fusioncore_config = devkit_fc_config if os.path.isfile(devkit_fc_config) \
                        else os.path.join(fusioncore_pkg, 'config', 'fusioncore.yaml')

    # ── Topological map ───────────────────────────────────────────────────────
    topo_nav_share = get_package_share_directory('topological_navigation')
    default_tmap2  = '/workspace/maps/mixed_test_map'
    tmap2_file     = os.getenv('TMAP2_FILE', default_tmap2)

    # ── Conditions ────────────────────────────────────────────────────────────
    gps_enabled  = PythonExpression(
        ["'", rover_port,  "' != 'virtual' and '", gps_type,  "' == 'ublox'"]
    )
    gps1_enabled = PythonExpression(
        ["'", rover1_port, "' != 'virtual' and '", gps1_type, "' == 'ublox'"]
    )
    # heading_shim requires RELPOSNED which only exists in dual-antenna (moving baseline) mode
    gps_dual_enabled = PythonExpression(
        ["'", rover_port,  "' != 'virtual' and '",
         rover1_port, "' != 'virtual' and '", gps_type, "' == 'ublox'"]
    )

    # ── ublox launch args ──────────────────────────────────────────────────────
    # mb+r mode: identify by serial string (dual-antenna, both serials programmed).
    # Single-antenna: ublox_single.launch.py reads GPS_USB_PATH_ROVER from env directly.
    rover_args = {'device_serial_string': rover_serial}
    base_args  = {'device_serial_string': rover1_serial}

    # ── Optional NTRIP corrections ─────────────────────────────────────────────
    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_client_node',
        name='ntrip_client',
        parameters=[ntrip_config],
        output='screen',
    ) if ntrip_available else None

    # ── Front receiver ─────────────────────────────────────────────────────────
    # Single-antenna: ublox_single.launch.py — ComposableNode, bus-path device,
    #                 load_config_view=False (skips VALGET sweep), namespace=rover.
    #                 Publishes /rover/ubx_nav_pvt and /rover/ublox_nav_sat_fix_hp.
    #                 No /rover/ubx_nav_rel_pos_ned (heading_shim not started).
    #
    # Dual-antenna:   ublox_mb+r_rover.launch.py — identified by device_serial_string.
    #                 Publishes /rover/ublox_nav_sat_fix_hp + /rover/ubx_nav_rel_pos_ned.
    front_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(devkit_launch_pkg, 'launch', 'ublox_single.launch.py')
                if single_antenna else
                os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_rover.launch.py')
            ),
            launch_arguments=({} if single_antenna else rover_args).items(),
        ),
    ]
    if ntrip_node:
        front_actions.append(ntrip_node)

    # ── Rear receiver: mb+r BASE ───────────────────────────────────────────────
    # Sends RTCM corrections to rover over UART2 (physical wire).
    # Not started in single-antenna mode (gps1_enabled=false).
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
    gps_fix_relay = Node(
        package='topic_tools',
        executable='relay',
        name='gps_fix_relay',
        arguments=['/rover/ublox_nav_sat_fix_hp', '/gnss/fix'],
        output='screen',
        condition=IfCondition(gps_enabled),
    )

    # ── Topic relay: /odom → /odom/wheels ─────────────────────────────────────
    # devkit_driver publishes on /odom; fusioncore hardcodes /odom/wheels.
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_wheels_relay',
        arguments=['/odom', '/odom/wheels'],
        output='screen',
    )

    # ── Heading shim: /rover/ubx_nav_rel_pos_ned → /gnss/heading ──────────────
    # RELPOSNED only exists in dual-antenna (moving baseline) mode.
    # Conditioned on gps_dual_enabled — not started for single-antenna.
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
        condition=IfCondition(gps_dual_enabled),
    )

    # ── fusioncore UKF ─────────────────────────────────────────────────────────
    # Single-antenna: fuses /gnss/fix + /odom/wheels only (no heading input).
    # Dual-antenna:   fuses /gnss/fix + /gnss/heading + /odom/wheels.
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

        # Front ublox:
        #   single-antenna → ublox_single.launch.py (bus-path, no VALGET)
        #   dual-antenna   → ublox_mb+r_rover.launch.py (serial-string)
        GroupAction(
            condition=IfCondition(gps_enabled),
            actions=front_actions,
        ),

        # Rear ublox — mb+r BASE (dual-antenna only)
        GroupAction(
            condition=IfCondition(gps1_enabled),
            actions=rear_actions,
        ),

        # /rover/ublox_nav_sat_fix_hp → /gnss/fix
        gps_fix_relay,

        # /odom → /odom/wheels
        odom_relay,

        # NAV-RELPOSNED → /gnss/heading (dual-antenna only)
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

        # Topological Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('topological_navigation'),
                    'launch', 'topological_navigation.launch.py'
                )
            ),
            launch_arguments={'map_path': tmap2_file}.items(),
        ),

        # Row following
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(row_follow_pkg, 'launch', 'row_follow.launch.py')
            ),
        ),
    ])
