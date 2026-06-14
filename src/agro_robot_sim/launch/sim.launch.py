import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    pkg_name     = "agro_robot_sim"
    pkg_share    = get_package_share_directory(pkg_name)
    gz_pkg_share = get_package_share_directory("ros_gz_sim")

    # 0. launch arguments
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.3")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="minha_fazenda.sdf",
        description="SDF world file name inside agro_robot_sim/worlds/",
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value="sowbot_01.xacro",
        description="URDF/xacro filename inside agro_robot_sim/urdf/",
    )

    # 1. open gazebo
    world = LaunchConfiguration("world")
    world_file = PathJoinSubstitution([pkg_share, "worlds", world])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r "), world_file],
            "on_exit_shutdown": "True",
        }.items(),
    )

    # 2. robot_state_publisher
    # use_sim_time: False — the nav stack (sim_nav.launch.py) runs entirely on
    # wall-clock time.  If RSP uses sim-time it stamps /tf at t=0 until Gazebo
    # starts publishing /clock; TF2 treats those as ancient and the nav stack
    # (wall-time) discards them.  Even after /clock flows, RSP sim-time ≠
    # nav-stack wall-time → lookup mismatches.  Wall-clock here keeps both
    # sides in the same time domain.
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", LaunchConfiguration("urdf")])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", xacro_file]), value_type=str
                ),
                "use_sim_time": False,
            }
        ],
    )

    # 3. spawn robot
    # 8 s delay: gz-sim loading maize.world takes 5–15 s on a typical laptop.
    # We use -topic /robot_description (published by robot_state_publisher above)
    # rather than -file xacro_file because gz sim create passes the file path
    # directly to the URDF parser, which does not invoke xacro — variables like
    # ${frame_len} are left unresolved and the parse fails.  robot_state_publisher
    # runs xacro internally and publishes the resolved URDF on /robot_description.
    spawn_entity = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot",
                output="screen",
                arguments=[
                    "-name",  "agro_robot",
                    "-topic", "/robot_description",
                    "-x", LaunchConfiguration("x"),
                    "-y", LaunchConfiguration("y"),
                    "-z", LaunchConfiguration("z"),
                ],
            )
        ],
    )

    # 4. topic bridge
    # The xacro declares bare plugin topics (cmd_vel, odom, tf, …) which Gazebo
    # scopes to /model/agro_robot/<topic> at runtime.  parameter_bridge argument
    # strings do NOT support gz topic remapping via the colon-suffix notation
    # ("gz.msgs.Twist:/model/agro_robot/cmd_vel") — that string is parsed as the
    # type name and produces "No template specialization for the pair".
    # Use the config_file parameter with a YAML mapping instead.
    #
    # 10 s delay: the bridge must not start until the model exists in Gazebo,
    # otherwise the /model/agro_robot/tf gz topic is absent and the bridge
    # logs "topic not found" and never subscribes.  2 s after the spawn timer
    # gives `gz sim create` time to complete and the model to initialise its
    # plugins before the bridge tries to connect.
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")
    ros_gz_bridge = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"config_file": bridge_config},
                ],
            )
        ],
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        world_arg,
        urdf_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])
