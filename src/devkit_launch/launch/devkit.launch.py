import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _nav2_nodes(params_file, real_condition):
    """Explicit Nav2 node declarations for real hardware.

    Replaces IncludeLaunchDescription of navigation_launch.py so we can
    omit collision_monitor, which requires observation_sources (a sensor)
    and throws InvalidParameterValueException on startup in Nav2 Jazzy when
    no sensor is configured — even with observation_sources: [] in the YAML.
    Re-add collision_monitor here once a lidar or ultrasonic is wired up.

    Remappings mirror those in navigation_launch.py:
      cmd_vel -> cmd_vel_nav  (controller_server, behavior_server)
      cmd_vel -> cmd_vel_nav  (velocity_smoother input, output stays cmd_vel)
    tf/tf_static are remapped globally via the common remappings list.
    """
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = dict(
        output='screen',
        condition=real_condition,
        parameters=[{'use_sim_time': False}, params_file],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings,
    )

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            condition=real_condition,
            parameters=[{'use_sim_time': False}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            **common,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            **common,
        ),
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            **common,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            condition=real_condition,
            parameters=[{'use_sim_time': False}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            **common,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            **common,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            condition=real_condition,
            parameters=[{'use_sim_time': False}, params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            **common,
        ),
        # collision_monitor intentionally omitted — no sensor available.
        # Add it back here with a proper params_file entry once a lidar
        # or ultrasonic is connected.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            condition=real_condition,
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                params_file,
            ],
        ),
    ]


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

    nav2_params = os.path.join(devkit_launch_pkg, 'config', 'nav2_params.yaml')

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
        # action servers and publishes map -> odom -> base_link TF.
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
        # Nodes declared explicitly (not via navigation_launch.py include) so
        # collision_monitor can be cleanly omitted until a sensor is available.
        # node_names in nav2_params.yaml controls which nodes the lifecycle
        # manager brings up — it must match the nodes declared here.
        TimerAction(period=2.0, actions=_nav2_nodes(nav2_params, real_condition)),

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

    
    ]


def generate_launch_description():
    ublox_pkg         = get_package_share_directory('ublox_dgnss')
    devkit_launch_pkg = get_package_share_directory('devkit_launch')
    ui_pkg            = get_package_share_directory('devkit_ui')

    rover_port   = os.getenv('GPS_PORT_ROVER',  'virtual')
    rover_serial = os.getenv('GPS_SERIAL_ROVER', '')
    gps_type     = os.getenv('GPS_TYPE_ROVER',  'ublox')

    rover1_port   = os.getenv('GPS_PORT_ROVER1', 'virtual')
    rover1_serial = os.getenv('GPS_SERIAL_ROVER1', '')
    gps1_type     = os.getenv('GPS_TYPE_ROVER1', 'ublox')

    mcu_port     = os.getenv('MCU_PORT',    'virtual')
    tmap2_file   = os.getenv('TMAP2_FILE',  '')

    fusioncore_config = os.path.join(devkit_launch_pkg, 'config', 'fusioncore.yaml')

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

    gps_enabled = PythonExpression(
        ["'", rover_port, "' != 'virtual' and '", gps_type, "' == 'ublox'"]
    )
    gps1_enabled = PythonExpression(
        ["'", rover1_port, "' != 'virtual' and '", gps1_type, "' == 'ublox'"]
    )

    front_args = {'device_family': 'F9P'}
    if rover_serial:
        front_args['device_serial_string'] = rover_serial

    rear_args = {'device_family': 'F9P'}
    if rover1_serial:
        rear_args['device_serial_string'] = rover1_serial

    # dual_antenna: True when a second F9P is present (moving-base receiver).
    # Computed the same way as gps1_enabled so they're always consistent.
    # gps1_enabled is a PythonExpression substitution (runtime); dual_antenna is
    # used as a plain Python bool at parse time for the course_over_ground param.
    # Both read the same env vars which don't change between parse and execution.
    dual_antenna = (rover1_port != 'virtual' and gps1_type == 'ublox')

    # NTRIP: gated by NTRIP_ENABLED env var, not file presence.
    # File presence is evaluated at build/install time in Docker images and
    # will always be True once ntrip.yaml is committed. Use an explicit env var
    # so operators can toggle NTRIP without touching the image.
    # To enable: set NTRIP_ENABLED=true in .env (fixusb.py writes this file).
    ntrip_config  = os.path.join(devkit_launch_pkg, 'config', 'ntrip.yaml')
    ntrip_enabled = os.getenv('NTRIP_ENABLED', 'false').lower() == 'true'

    return LaunchDescription([
        sim_arg,

        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # ── GPS hardware ─────────────────────────────────────────────────────

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

        # ── NTRIP corrections (u-blox path) ──────────────────────────────────
        # Delivers RTCM to ublox_dgnss when NTRIP_ENABLED=true in .env.
        # Fill in config/ntrip.yaml (host/port/mountpoint/credentials) then
        # set NTRIP_ENABLED=true and re-run fixusb.py to activate.
        # Requires: sudo apt install ros-jazzy-ntrip-client
        *([Node(
            package='ntrip_client',
            executable='ntrip_client_node',
            name='ntrip_client',
            parameters=[ntrip_config],
            output='screen',
        )] if (ntrip_enabled and rover_port != 'virtual' and gps_type == 'ublox') else []),

        # ── Topic relays & GNSS shims ─────────────────────────────────────────
        # rtk_navsatfix_shim replaces the dumb topic_tools relay.
        # It combines /rover/ublox_nav_sat_fix_hp + /rover/ubx_nav_pvt.carr_soln
        # to emit /gnss/fix with a status that correctly reflects RTK FLOAT vs
        # FIXED (ublox_dgnss reports STATUS_GBAS_FIX for both without this shim).
        # Only meaningful when GPS hardware is present; in sim /gnss/fix is unused.
        GroupAction(
            condition=IfCondition(gps_enabled),
            actions=[
                Node(
                    package='devkit_driver',
                    executable='rtk_navsatfix_shim',
                    name='rtk_navsatfix_shim',
                    output='screen',
                ),
            ],
        ),

        Node(
            package='topic_tools',
            executable='relay',
            name='odom_wheels_relay',
            arguments=['/odom', '/odom/wheels'],
            output='screen',
        ),

        # relposned_heading_shim: dual-antenna RTK heading from NAV-RELPOSNED.
        # Only launched when a second F9P (moving base) is present.
        GroupAction(
            condition=IfCondition(gps1_enabled),
            actions=[
                Node(
                    package='devkit_driver',
                    executable='relposned_heading_shim',
                    name='relposned_heading_shim',
                    output='screen',
                ),
            ],
        ),

        # course_over_ground: single-F9P CoG heading fallback.
        # Launched whenever GPS is present. dual_antenna param disables
        # publishing when the relposned shim is active, preventing two nodes
        # from racing on /gnss/heading.
        GroupAction(
            condition=IfCondition(gps_enabled),
            actions=[
                Node(
                    package='devkit_driver',
                    executable='course_over_ground',
                    name='course_over_ground',
                    parameters=[{'dual_antenna': dual_antenna}],
                    output='screen',
                ),
            ],
        ),

        # ── FusionCore UKF ───────────────────────────────────────────────────
        # Launched directly — NOT wrapped in a lifecycle_manager.
        #
        # fusioncore does NOT implement the bond heartbeat protocol that
        # nav2_lifecycle_manager requires, so wrapping it causes a guaranteed
        # bond timeout on every boot regardless of the timeout value set.
        #
        # fusioncore was documented as self-transitioning configure → active,
        # but in practice it stalls at unconfigured — likely because /gnss/fix
        # and /odom/wheels are not yet available when the node initialises.
        # We drive the two lifecycle transitions explicitly via a timed
        # ExecuteProcess pair. Timings are deliberately conservative because
        # the configure callback does non-trivial work (PROJ transform, UKF
        # init, parameter validation) and activate must not race ahead of it.
        # Earlier 4 s / 5 s spacing produced a "Transitioning successful" but
        # left the node in inactive a few seconds later — bumped to 8 s / 14 s
        # to give configure time to finish before activate is issued.
        Node(
            package='fusioncore_ros',
            executable='fusioncore_node',
            name='fusioncore',
            parameters=[fusioncore_config],
            output='screen',
        ),
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/fusioncore', 'configure'],
                output='screen',
            ),
        ]),
        TimerAction(period=14.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/fusioncore', 'activate'],
                output='screen',
            ),
        ]),

        # ── Static map -> odom TF (real hardware only) ───────────────────────
        # In sim, fake_nav2_server owns the full map->odom->base_link TF chain.
        # In real mode, fusioncore publishes odom->base_link; this node closes
        # the chain with an identity map->odom.
        # TODO: replace with GPS-anchored origin once tmap2 nodes are surveyed.
        Node(
            condition=real_condition,
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),

        # ── Static odom -> base_link TF (sim / no-MCU only) ──────────────────
        # In sim mode the MCU is virtual so odom_handler never runs and
        # fusioncore has no wheel odom to fuse, meaning odom->base_link is
        # never published. This identity static TF closes the chain so that
        # topological localisation and Nav2 can start up.
        # In real mode fusioncore owns this TF — do not publish it there.
        Node(
            condition=sim_condition,
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_static',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # ── Static base_link -> imu_link TF (no MCU only) ────────────────────
        # When MCU is virtual there is no IMU driver publishing this transform.
        # Fusioncore requires it to exist before it will activate. An identity
        # transform is correct for bench/dev — assumes IMU is at robot centre.
        # When a real MCU is present the IMU driver owns this TF; do not
        # compete with it.
        Node(
            condition=IfCondition(
                PythonExpression(["'", mcu_port, "' == 'virtual'"])
            ),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link_static',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen',
        ),

        # ── Devkit Driver ────────────────────────────────────────────────────
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

        # ── Row Follow action server (real hardware only) ────────────────────
        # Provides the /limbic_row_follow action server that topo_nav calls on
        # limbic_row_follow edge types. Omitted in sim: fake_nav2_server handles
        # /navigate_to_pose directly and there is no Neo SBC to talk to.
        # Including it in sim would cause immediate timeouts on every row edge
        # (it waits for Neo's /row_follow/enable service which doesn't exist).
        GroupAction(
            condition=real_condition,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('sowbot_row_follow'),
                            'launch', 'row_follow.launch.py'
                        )
                    ),
                ),
            ],
        ),

        # ── Topological Navigation ───────────────────────────────────────────
    ] + _topo_nav_nodes(tmap2_file, sim_condition, real_condition, devkit_launch_pkg))
