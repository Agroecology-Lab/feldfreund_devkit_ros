import os
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
        SetEnvironmentVariable('LOAD_CONFIG_VIEW', 'false'),  # skip VALGET write-back

        ComposableNodeContainer(
            name='ublox_container',
            namespace='rover',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ublox_dgnss_node',
                    plugin='ublox_dgnss::UbloxDGNSSNode',
                    name='ublox_dgnss',
                    namespace='rover',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    ])
