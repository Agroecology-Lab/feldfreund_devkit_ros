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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nav2_nodes(params_file, use_sim_time):
    """Explicit Nav2 node declarations — runs in BOTH sim and real-hardware
    modes now (previously real-hardware only, gated by real_condition).

    use_sim_time is the 'sim' LaunchConfiguration substitution (a string
    'true'/'false', not resolved until launch time) wrapped in ParameterValue
    so Nav2 receives an actual bool parameter rather than the literal string
    'true'/'false'. It is NOT a plain Python bool — that would require
    knowing sim/real at generate_launch_description() parse time, which we
    don't: 'sim' itself is a LaunchConfiguration, only resolved at runtime.
    Real Nav2 is now always the navigation backend; in sim mode its
    use_sim_time=True nodes simply sit waiting for a valid /clock until
    Gazebo is started via one of the UI buttons (Launch World/Spawn Robot or
    Launch Sim) — exactly the same wait fake_nav2_server's clock stub already
    bridges for topo nav today. See _kill_fake_nav2_action() below for why
    fake_nav2_server's own action servers must stop once that happens.

    Replaces IncludeLaunchDescription of navigation_launch.py so we can
    omit collision_monitor, which requires observation_sources (a sensor)
    and throws InvalidParameterValueException on startup in Nav2 Jazzy when
    no sensor is configured — even with observation_sources: [] in the YAML.
    Re-add collision_monitor here once a lidar or ultrasonic is wired up.

    Remappings mirror those in navigation_launch.py:
      cmd_vel -> cmd_vel_nav  (controller_server, behavior_server)
      cmd_vel -> cmd_vel_nav  (velocity_smoother input), cmd_vel_smoothed -> cmd_vel  (velocity_smoother output)
    tf/tf_static are remapped globally via the common remappings list.
    """
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {
        "output": 'screen',
        "parameters": [{'use_sim_time': use_sim_time_param}, params_file],
        "arguments": ['--ros-args', '--log-level', 'info'],
        "remappings": remappings,
    }

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}, params_file],
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
            parameters=[{'use_sim_time': use_sim_time_param}, params_file],
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
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}, params_file],
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
            parameters=[
                {'use_sim_time': use_sim_time_param},
                {'autostart': True},
                params_file,
            ],
        ),
    ]


def _kill_fake_nav2_action(sim_condition):
    """Kill fake_nav2_server once real Nav2's lifecycle manager has actually
    activated bt_navigator — both run a NavigateToPose action server and
    would otherwise collide on the same action name.

    fake_nav2_server itself keeps starting unconditionally in sim mode (see
    the node below) because its wall-time /clock publisher is still needed:
    real Nav2 launches at container boot here, well before Gazebo exists, so
    something has to keep /clock and map->odom->base_link TF alive in the
    meantime or Nav2's lifecycle/TF lookups fail outright. Once Gazebo's
    ros_gz_bridge comes up (via any of the UI's Launch World/Spawn
    Robot/Launch Sim buttons) its RELIABLE /clock wins by QoS precedence
    automatically — but fake_nav2_server's ACTION SERVERS don't get
    superseded that way, they just keep running and competing with
    bt_navigator. So we kill the process outright, on a poll loop, the
    moment bt_navigator is actually active.

    BOUNDED, NOT INFINITE: bt_navigator activating depends on
    lifecycle_manager_navigation's autostart bring-up succeeding, which is
    exactly the kind of thing that's failed silently before in this codebase
    (see the lifecycle-manager race this whole launch-file split was built
    to fix). An earlier version of this watchdog looped `while true` with no
    timeout — if bt_navigator never activates, that version would poll
    forever with zero visible signal that anything was wrong, while
    fake_nav2_server kept masking the failure by staying alive and looking
    functional. This version caps at MAX_ATTEMPTS, logs progress every 10
    attempts so it's visible in the boot log rather than silent, and on
    timeout logs a loud, unambiguous ERROR and exits non-zero — making
    "real Nav2 never came up" a visible failure instead of a quiet one.

    Implemented as a plain bash poll loop rather than a custom node —
    topological_nav_simulator (which owns fake_nav2_server) isn't a package
    we maintain here, so adding a new executable to it isn't an option from
    this launch file.
    """
    poll_interval_s = 2
    max_attempts = 90   # 90 * 2s = 3 minutes — generous; Gazebo is NOT
                        # required for bt_navigator to activate, only real
                        # Nav2's own lifecycle bring-up, so this should
                        # resolve quickly if it's going to resolve at all.
    return ExecuteProcess(
        condition=sim_condition,
        cmd=[
            '/bin/bash', '-c',
            (
                f'ATTEMPT=0; '
                f'while [ "$ATTEMPT" -lt {max_attempts} ]; do '
                f'sleep {poll_interval_s}; '
                f'ATTEMPT=$((ATTEMPT + 1)); '
                f'STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null); '
                f'if [[ "$STATE" == active* ]]; then '
                f'pkill -f fake_nav2_server || true; '
                f'echo "[kill_fake_nav2] bt_navigator active after '
                f'$((ATTEMPT * {poll_interval_s}))s — killed fake_nav2_server"; '
                f'exit 0; '
                f'fi; '
                f'if [ $((ATTEMPT % 10)) -eq 0 ]; then '
                f'echo "[kill_fake_nav2] still waiting for bt_navigator to '
                f'activate ($((ATTEMPT * {poll_interval_s}))s elapsed, '
                f'last state: ${{STATE:-unknown}})"; '
                f'fi; '
                f'done; '
                f'echo "[kill_fake_nav2] ERROR: bt_navigator did not '
                f'activate within $(({max_attempts} * {poll_interval_s}))s '
                f'— real Nav2 lifecycle bring-up appears stuck. '
                f'fake_nav2_server left running as a fallback. '
                f'Check lifecycle_manager_navigation and bt_navigator logs." '
                f'>&2; '
                f'exit 1'
            ),
        ],
        name='kill_fake_nav2_on_bt_active',
        output='screen',
    )


def _topo_nav_nodes(tmap2_file, sim, sim_condition, devkit_bringup_pkg):
    """Inlined topo nav stack with real Nav2 as the navigation backend in
    BOTH sim and real-hardware modes.

    Previously fake_nav2_server (action servers + clock/TF stub) was the
    sim-mode backend and real Nav2 was hardware-only. Real Nav2 is now
    always the navigation backend; fake_nav2_server still starts in sim
    mode (unconditionally, alongside real Nav2) purely to bridge /clock and
    map->odom->base_link TF until Gazebo exists, and gets killed by
    _kill_fake_nav2_action() the moment bt_navigator actually activates.
    See that function's docstring for the full rationale.

    sim is the 'sim' LaunchConfiguration itself (not yet resolved to a bool)
    — passed straight through to _nav2_nodes() as use_sim_time, since Nav2's
    use_sim_time param needs exactly this same true/false value.
    """
    topo_share = get_package_share_directory('topological_navigation')
    map_path = tmap2_file or os.path.join(
        topo_share, 'config', 'mixed_actions_map.yaml')

    nav2_params = os.path.join(devkit_bringup_pkg, 'config', 'nav2_params.yaml')

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

        # 3a. fake_nav2_server — SIM ONLY, /clock + TF bridge until Gazebo
        # exists. Its NavigateToPose/etc action servers are a stopgap, not
        # the real backend — see _kill_fake_nav2_action() below, which kills
        # this process once real Nav2's bt_navigator is actually active.
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

        # 3b. fake_nav2_server kill watchdog — SIM ONLY. No-op on real
        # hardware (condition never fires there since fake_nav2_server
        # never starts).
        _kill_fake_nav2_action(sim_condition),

        # 3c. Real Nav2 — now the navigation backend in BOTH sim and real
        # hardware modes (previously real-hardware only). use_sim_time is
        # the 'sim' LaunchConfiguration itself, resolved at launch time;
        # node_names in nav2_params.yaml controls which nodes the lifecycle
        # manager brings up — it must match the nodes declared in
        # _nav2_nodes().
        TimerAction(period=2.0, actions=_nav2_nodes(
            nav2_params, use_sim_time=sim)),

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
    devkit_bringup_pkg = get_package_share_directory('devkit_bringup')
    ui_pkg            = get_package_share_directory('devkit_ui')

    rover_port   = os.getenv('GPS_PORT_ROVER',  'virtual')
    rover_serial = os.getenv('GPS_SERIAL_ROVER', '')
    gps_type     = os.getenv('GPS_TYPE_ROVER',  'ublox')

    rover1_port   = os.getenv('GPS_PORT_ROVER1', 'virtual')
    rover1_serial = os.getenv('GPS_SERIAL_ROVER1', '')
    gps1_type     = os.getenv('GPS_TYPE_ROVER1', 'ublox')

    mcu_port     = os.getenv('MCU_PORT',    'virtual')
    tmap2_file   = os.getenv('TMAP2_FILE',  '')

    fusioncore_config = os.path.join(devkit_bringup_pkg, 'config', 'fusioncore.yaml')

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
    ntrip_config  = os.path.join(devkit_bringup_pkg, 'config', 'ntrip.yaml')
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
        # autostart:true (default in fusioncore >= 0.3.1) makes the node
        # self-transition configure -> activate ~200ms after on_configure()
        # returns. No external lifecycle management needed.
        Node(
            package='fusioncore_ros',
            executable='fusioncore_node',
            name='fusioncore',
            parameters=[fusioncore_config],
            output='screen',
        ),

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

        # ── Static odom -> base_link TF (no real MCU) ────────────────────────
        # Needed whenever the MCU is virtual: odom_handler never runs (no MCU)
        # and on the bench fusioncore has no RTK fix to fuse, so neither owns
        # odom->base_link and the TF tree splits into two unconnected trees
        # (map->odom and a detached base_link->imu_link). That breaks
        # topological localisation (current node stays unknown) and Nav2.
        # This identity static TF closes the chain.
        #
        # Gate = MCU virtual AND not sim: in true sim mode fake_nav2_server
        # already owns the full map->odom->base_link chain, so publishing here
        # too would be a duplicate broadcaster. In real mode WITH a real MCU,
        # odom_handler / fusioncore own this TF — do not compete.
        Node(
            condition=IfCondition(
                PythonExpression(
                    ["'", mcu_port, "' == 'virtual' and '", sim, "' != 'true'"]
                )
            ),
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
                os.path.join(devkit_bringup_pkg, 'launch', 'devkit_driver.launch.py')
            ),
            launch_arguments={'port': mcu_port}.items(),
        ),

        # ── Devkit UI ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ui_pkg, 'launch', 'ui.launch.py')
            ),
            launch_arguments={'sim': sim}.items(),
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

        # ── Sentor System Health Monitoring (real hardware only) ─────────────
        # LCAS Sentor publishes /safety/heartbeat (estop/bumper + devkit_driver
        # node) and /warning/heartbeat (camera/odom/battery/neo_vision
        # perception). Gated to real hardware: the safety topics (/estop/*,
        # /bumper/*) and the monitored nodes (devkit_driver, usb_cam,
        # crop_row_node) do not exist in sim, so the safety beat would sit
        # false continuously. Config installs to share/devkit_bringup/config via
        # the install(DIRECTORY ... config) rule in CMakeLists.txt.
        # NOTE: executable is sentor_node.py (scripts/), NOT test_sentor.py.
        # The heartbeat publish rate / timeout are owned by SafetyMonitor's own
        # node and are not parameterisable from here, so no rate params passed.
        GroupAction(
            condition=real_condition,
            actions=[
                Node(
                    package='sentor',
                    executable='sentor_node.py',
                    name='sentor',
                    output='screen',
                    parameters=[{
                        'config_file': os.path.join(
                            devkit_bringup_pkg, 'config', 'sowbot_monitor.yaml'),
                    }],
                ),
            ],
        ),

        # ── Topological Navigation ───────────────────────────────────────────
    ] + _topo_nav_nodes(tmap2_file, sim, sim_condition, devkit_bringup_pkg))
