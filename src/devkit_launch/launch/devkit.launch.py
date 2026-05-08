import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _topo_nav_nodes(tmap2_file, sim_condition, real_condition, devkit_launch_pkg):
    """Inlined topo nav stack with sim/real conditional Nav2 backends.
    
    Replaces the upstream topological_navigation.launch.py which hardcodes
    fake_nav2_server unconditionally. Inlining lets us route to either
    fake_nav2_server (sim) or real Nav2 (hardware).
    """
    topo_share = get_package_share_directory('topological_navigation')
    rviz_cfg = os.path.join(topo_share, 'rviz', 'topological_navigation.rviz')
    map_path = tmap2_file or os.path.join(
        topo_share, 'config', 'mixed_actions_map.yaml')

    return [
        # 1. Map Manager — loads and publishes the topological map
        Node(
            package='topological_navigation',
            executable='map_manager2.py',
            name='topological_map_manager_2',
            output='screen',
            arguments=[map_path],
        ),

        # 2. Localisation (delayed 2 s so map is published first)
        TimerAction(period=2.0, actions=[
            Node(
                package='topological_navigation',
                executable='localisation2.py',
                name='topological_localisation',
                output='screen',
            ),
        ]),

        # 3a. Fake Nav2 simulator — SIM ONLY
        # Provides /navigate_to_pose, /navigate_through_poses, /follow_waypoints
        # action servers and publishes map → odom → base_link TF.
        TimerAction(period=2.0, actions=[
            Node(
                condition=sim_condition,
                package='topological_nav_simulator',
                executable='fake_nav2_server',
                name='fake_nav2_server',
                output='screen',
                parameters=[{
                    'initial_x': 0.0,
                    'initial_y': 0.0,
                    'initial_yaw': 0.0,
                }],
            ),
        ]),

        # 3b. Real Nav2 — REAL HARDWARE ONLY
        # Brings up the full Nav2 stack (controller, planner, BT navigator,
        # behavior server, waypoint follower, smoother, velocity smoother,
        # collision monitor, lifecycle manager). Uses the project's nav2
        # params file. Does NOT include localisation — fusioncore provides
        # odom → base_link and a static transform provides map → odom.
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py'
                    )
                ),
                condition=real_condition,
                launch_arguments={
                    'use_sim_time': 'false',
                    'autostart': 'true',
                    'params_file': os.path.join(
                        devkit_launch_pkg, 'config', 'nav2_params.yaml'),
                }.items(),
            ),
        ]),

        # 4. Topological navigation server (delayed 3 s)
        TimerAction(period=3.0, actions=[
            Node(
                package='topological_navigation',
                executable='navigation2.py',
                name='topological_navigation',
                output='screen',
            ),
        ]),

        # 5. Topological map visualiser (delayed 4 s)
        TimerAction(period=4.0, actions=[
            Node(
                package='topological_navigation_visual',
                executable='topological_map_visualiser.py',
                name='topological_map_visualiser',
                output='screen',
                parameters=[{'edit_mode': True}],
            ),
        ]),

        # 6. RViz (delayed 5 s)
        TimerAction(period=5.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_cfg],
            ),
        ]),
    ]


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
    gps1_type     = os.getenv('GPS_TYPE_ROVER1', 'ublox')

    mcu_port     = os.getenv('MCU_PORT',    'virtual')
    tmap2_file   = os.getenv('TMAP2_FILE',  '')

    fusioncore_config = os.path.join(devkit_launch_pkg, 'config', 'fusioncore.yaml')

    # ── Sim/real mode decision ──────────────────────────────────────────────
    # manage.py passes sim:=true/false on the command line. This launch file
    # consumes it and uses it to gate fake_nav2_server (sim only) and the
    # static map→odom TF (real only).
    #
    # Default mirrors manage.py's auto-detect: if any hardware is present the
    # default is real mode, otherwise sim. This makes direct ros2 launch
    # invocations behave sensibly without needing manage.py.
    any_hw_present = (rover_port  != 'virtual' or
                      rover1_port != 'virtual' or
                      mcu_port    != 'virtual')
    is_sim_default = 'false' if any_hw_present else 'true'

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value=is_sim_default,
        description='true = sim mode (fake_nav2_server, fake TF), '
                    'false = real hardware (real Nav2)',
    )
    sim = LaunchConfiguration('sim')
    sim_condition  = IfCondition(sim)
    real_condition = IfCondition(PythonExpression(["'", sim, "' != 'true'"]))

    # Gate GPS groups on physical hardware being present and type being ublox
    gps_enabled = PythonExpression(
        ["'", rover_port, "' != 'virtual' and '", gps_type, "' == 'ublox'"]
    )
    gps1_enabled = PythonExpression(
        ["'", rover1_port, "' != 'virtual' and '", gps1_type, "' == 'ublox'"]
    )

    # Rover F9P (front antenna) — moving-base rover mode
    front_args = {'device_family': 'F9P'}
    if rover_serial:
        front_args['device_serial_string'] = rover_serial

    # Base F9P (rear antenna) — moving-base base mode
    rear_args = {'device_family': 'F9P'}
    if rover1_serial:
        rear_args['device_serial_string'] = rover1_serial

    return LaunchDescription([
        sim_arg,

        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # ── GPS hardware ─────────────────────────────────────────────────────

        # Base F9P (rear) — start before rover so RTCM is ready on UART2
        GroupAction(
            condition=IfCondition(gps1_enabled),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_base.launch.py')
                    ),
                    launch_arguments=rear_args.items(),
                ),
            ],
        ),

        # Rover F9P (front) — moving-base rover, produces NavSatFix + RELPOSNED
        GroupAction(
            condition=IfCondition(gps_enabled),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ublox_pkg, 'launch', 'ublox_mb+r_rover.launch.py')
                    ),
                    launch_arguments=front_args.items(),
                ),
            ],
        ),

        # ── Topic relays (always run; idle silently if source absent) ────────

        # /rover/fix → /gnss/fix
        Node(
            package='topic_tools',
            executable='relay',
            name='navsatfix_relay',
            arguments=['/rover/fix', '/gnss/fix'],
            output='screen',
        ),

        # /odom → /odom/wheels (explicit fusioncore odom input)
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_wheels_relay',
            arguments=['/odom', '/odom/wheels'],
            output='screen',
        ),

        # NAV-RELPOSNED → /gnss/heading (sensor_msgs/Imu, ENU yaw quaternion)
        # Rejects silently if dual antenna not connected.
        Node(
            package='devkit_driver',
            executable='relposned_heading_shim',
            name='relposned_heading_shim',
            output='screen',
        ),

        # ── FusionCore UKF ───────────────────────────────────────────────────
        # Always publishes odom → base_link TF.
        # Sim: wheel odom only (no GNSS fix from real hardware).
        # Real: GNSS fix + heading + wheel odom.
        Node(
            package='fusioncore_ros',
            executable='fusioncore_node',
            name='fusioncore',
            parameters=[fusioncore_config],
            output='screen',
        ),

        # ── Static map → odom TF (real hardware only) ────────────────────────
        # In sim, fake_nav2_server provides this. In real mode, broadcast
        # identity so the topo nav TF chain (map → odom → base_link) is
        # complete.
        # TODO: replace with GPS-anchored map origin once tmap2 is surveyed.
        Node(
            condition=real_condition,
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),

        # ── Devkit Driver (Lizard ESP32 bridge) ──────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(devkit_launch_pkg, 'launch', 'devkit_driver.launch.py')
            ),
            launch_arguments={'port': mcu_port}.items(),
        ),

        # ── Devkit UI ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ui_pkg, 'launch', 'ui.launch.py')
            ),
        ),

        # ── Topological Navigation (inlined, conditional fake_nav2) ──────────
    ] + _topo_nav_nodes(tmap2_file, sim_condition, real_condition, devkit_launch_pkg))
