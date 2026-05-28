import os
# SUPERSEDED — this file is no longer used by devkit.launch.py.
# devkit.launch.py conditionally includes ublox_dgnss/launch/ublox_mb+r_rover.launch.py
# (single-F9P, no moving base) or ublox_mb+r_base + ublox_mb+r_rover (dual-F9P RTK)
# based on GPS_PORT_ROVER / GPS_PORT_ROVER1 env vars set by fixusb.py.
# Kept for standalone bench testing only. Do not wire into any production launch.
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    usb_path = os.getenv('GPS_USB_PATH_ROVER', 'virtual')

    return LaunchDescription([
        # ublox_dgnss reads uppercase env vars, not ROS parameters.
        # DEVICE_SERIAL_STRING is intentionally omitted — this F9P has iSerial=0
        # so the driver falls back to "use first F9P device found" automatically.
        SetEnvironmentVariable('DEVICE_FAMILY',    'F9P'),
        SetEnvironmentVariable('FRAME_ID',         'gnss'),
        # Skip CFG-VALGET read-back verification after writing params.
        # Required for firmware < HPG 1.32: params like CFG_SEC_JAMDET_SENSITIVITY_HI,
        # CFG_SIGNAL_GAL_*, CFG_SIGNAL_BDS_* are unknown to older firmware and never
        # ACK the VALGET, causing a 5-second batch timeout and degraded-mode startup.
        SetEnvironmentVariable('LOAD_CONFIG_VIEW', 'false'),

        ComposableNodeContainer(
            name='ublox_container',
            namespace='rover',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # F9P USB driver — publishes raw UBX messages
                ComposableNode(
                    package='ublox_dgnss_node',
                    plugin='ublox_dgnss::UbloxDGNSSNode',
                    name='ublox_dgnss',
                    namespace='rover',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                # Converts UBX NAV-HPPOSLLH → sensor_msgs/NavSatFix HP
                # Publishes /rover/ublox_nav_sat_fix_hp which devkit.launch.py
                # relays to /gnss/fix for fusioncore.
                ComposableNode(
                    package='ublox_nav_sat_fix_hp_node',
                    plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                    name='ublox_nav_sat_fix_hp',
                    namespace='rover',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    ])
