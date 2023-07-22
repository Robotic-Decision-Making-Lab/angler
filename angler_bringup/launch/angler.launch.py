# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to run the Angler system.

    Returns:
        The Angler ROS 2 launch description.
    """
    args = [
        DeclareLaunchArgument(
            "description_package",
            default_value="angler_description",
            description=(
                "The description package with the angler configuration files. This is"
                " typically not set, but is available in case another description"
                " package has been defined."
            ),
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="angler.config.xacro",
            description="The URDF/XACRO for the UVMS.",
        ),
        DeclareLaunchArgument(
            "planning_file",
            default_value="planning.yaml",
            description="The Angler planning configuration file.",
        ),
        DeclareLaunchArgument(
            "mux_file",
            default_value="mux.yaml",
            description="The Angler mux + demux configuration file.",
        ),
        DeclareLaunchArgument(
            "localization_file",
            default_value="localization.yaml",
            description="The BlueROV2 Heavy localization configuration file.",
        ),
        DeclareLaunchArgument(
            "manager_file",
            default_value="manager.yaml",
            description="The BlueROV2 Heavy manager configuration file.",
        ),
        DeclareLaunchArgument(
            "mavros_file",
            default_value="mavros.yaml",
            description="The MAVROS configuration file.",
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="angler_underwater.world",
            description="The world configuration to load if using Gazebo.",
        ),
        DeclareLaunchArgument(
            "ardusub_params_file",
            default_value="angler.parm",
            description=(
                "The ArduSub parameters that the BlueROV2 should use if running in"
                " simulation."
            ),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="angler_controllers.yaml",
            description="The BlueROV2 Heavy controller configuration file.",
        ),
        DeclareLaunchArgument(
            "manipulator_controller",
            default_value="forward_velocity_controller",
            description="The ros2_control controller to use with the manipulator(s).",
            choices=[
                "forward_velocity_controller",
                "forward_position_controller",
                "feedback_joint_position_trajectory_controller",
                "feedback_joint_velocity_trajectory_controller",
            ],
        ),
        DeclareLaunchArgument(
            "base_controller",
            default_value="ismc",
            description=(
                "The ROV controller to use; this should be the same name as the"
                " controller's executable."
            ),
            choices=["ismc"],
        ),
        DeclareLaunchArgument(
            "localization_source",
            default_value="gazebo",
            choices=["mocap", "camera", "gazebo"],
            description="The localization source to stream from.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description=(
                "Launch the BlueROV2 camera stream. This is automatically set to true"
                " when using the camera for localization."
            ),
        ),
        DeclareLaunchArgument(
            "use_mocap",
            default_value="false",
            description=(
                "Launch the Qualisys motion capture stream. This is automatically"
                " set to true when using the motion capture system for localization."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", description="Launch RViz2."
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="view_angler.rviz",
            description="The RViz2 configuration file.",
        ),
        DeclareLaunchArgument(
            "manipulator_serial_port",
            default_value="''",
            description=(
                "The serial port that the Alpha 5 is available at (e.g., /dev/ttyUSB0)."
            ),
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description=(
                "The configuration file that defines the initial positions used for"
                " the Reach Alpha 5 in simulation."
            ),
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "The prefix of the BlueROV2. This is useful for multi-robot setups."
                " Expected format '<prefix>/'."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description=(
                "The namespace of the launched nodes. This is useful for multi-robot"
                " setups. If the namespace is changed, then the namespace in the"
                " controller configuration must be updated. Expected format '<ns>/'."
            ),
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description=(
                "Start the Reach Alpha 5 driver using fake hardware mirroring command"
                " to its states. If this is set to 'false', the Alpha 5 manipulator"
                " serial port should be specified."
            ),
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    use_sim = LaunchConfiguration("use_sim")
    controllers_file = LaunchConfiguration("controllers_file")
    use_rviz = LaunchConfiguration("use_rviz")
    namespace = LaunchConfiguration("namespace")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    manipulator_controller = LaunchConfiguration("manipulator_controller")

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "urdf",
                        LaunchConfiguration("description_file"),
                    ]
                ),
                " ",
                "prefix:=",
                LaunchConfiguration("prefix"),
                " ",
                "use_fake_hardware:=",
                LaunchConfiguration("use_fake_hardware"),
                " ",
                "use_sim:=",
                use_sim,
                " ",
                "serial_port:=",
                LaunchConfiguration("manipulator_serial_port"),
                " ",
                "controllers_file:=",
                controllers_file,
                " ",
                "initial_positions_file:=",
                initial_positions_file,
                " ",
                "namespace:=",
                namespace,
            ]
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "config",
                    controllers_file,
                ]
            ),
        ],
        condition=UnlessCondition(use_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            [namespace, "controller_manager"],
        ],
        parameters=[{"use_sim_time": use_sim}],
    )

    manipulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[
            manipulator_controller,
            "--controller-manager",
            [namespace, "controller_manager"],
        ],
        parameters=[{"use_sim_time": use_sim}],
    )

    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "angler",
            "-topic",
            "robot_description",
        ],
        output="both",
        condition=IfCondition(use_sim),
        parameters=[{"use_sim_time": use_sim}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "rviz",
                    LaunchConfiguration("rviz_config"),
                ]
            ),
        ],
        parameters=[robot_description, {"use_sim_time": use_sim}],
        condition=IfCondition(use_rviz),
    )

    nodes = [
        gz_spawner,
        control_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawner,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[manipulator_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner, on_exit=[rviz]
            ),
            condition=IfCondition(use_rviz),
        ),
        Node(
            package="mavros",
            executable="mavros_node",
            output="both",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("mavros_file"),
                    ]
                ),
                {"use_sim_time": use_sim},
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                # Clock (IGN -> ROS 2)
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                # Odom (IGN -> ROS 2)
                "/model/angler/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            output="screen",
            condition=IfCondition(use_sim),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": use_sim}],
        ),
        Node(
            package="angler_utils",
            executable="initial_position_setter",
            name="initial_position_setter",
            parameters=[
                {
                    "initial_positions_file": PathJoinSubstitution(
                        [description_package, "config", initial_positions_file]
                    ),
                    "controller_cmd_topic": "/forward_velocity_controller/commands",
                }
            ],
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        use_sim,
                        "' == 'true' and '",
                        manipulator_controller,
                        "' == 'forward_velocity_controller'",
                    ]
                )
            ),
        ),
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments=[
                (
                    "gz_args",
                    [
                        "-v",
                        "4",
                        " ",
                        "-r",
                        " ",
                        LaunchConfiguration("gazebo_world_file"),
                    ],
                )
            ],
            condition=IfCondition(use_sim),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("angler_bringup"),
                        "launch",
                        "static_tf_broadcasters.launch.py",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("angler_planning"), "planning.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("planning_file"),
                    ]
                )
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("angler_mux"), "mux.launch.py"])
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("mux_file"),
                    ]
                )
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_manager"), "manager.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("manager_file"),
                    ]
                ),
                "use_sim_time": use_sim,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_control"), "launch", "control.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        controllers_file,
                    ]
                ),
                "controller": LaunchConfiguration("base_controller"),
                "use_sim_time": use_sim,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_localization"), "localization.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("localization_file"),
                    ]
                ),
                "localization_source": LaunchConfiguration("localization_source"),
                "use_mocap": LaunchConfiguration("use_mocap"),
                "use_camera": LaunchConfiguration("use_camera"),
                "use_sim_time": use_sim,
            }.items(),
        ),
    ]

    processes = [
        ExecuteProcess(
            cmd=[
                "ardusub",
                "-S",
                "-w",
                "-M",
                "JSON",
                "--defaults",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "ardusub",
                        LaunchConfiguration("ardusub_params_file"),
                    ]
                ),
                "-I0",
                "--home",
                "44.65870,-124.06556,0.0,270.0",  # my not-so-secret surf spot
            ],
            output="screen",
            condition=IfCondition(use_sim),
        ),
    ]

    return LaunchDescription(args + nodes + includes + processes)
