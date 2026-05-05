import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    usb_path = os.getenv('GPS_USB_PATH_ROVER', 'virtual')

    return LaunchDescription([
        ComposableNodeContainer(
            name='ublox_container',
            namespace='rover',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ublox_dgnss',
                    plugin='ublox_dgnss::UbloxDgnssDrv',
                    name='ublox_dgnss',
                    namespace='rover',
                    parameters=[{
                        'device':           usb_path,
                        'load_config_view': False,
                        'frame_id':         'gnss',
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    ])
