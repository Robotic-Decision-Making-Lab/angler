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
    """Generate a launch description to run the BlueROV2 Heavy Alpha system.

    Returns:
        The BlueROV2 Heavy Alpha ROS 2 launch description.
    """
    # Set some constants for this configuration
    description_package = "angler_description"
    configuration_type = "bluerov2_heavy_alpha"
    description_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "xacro",
            configuration_type,
            "config.xacro",
        ]
    )
    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            configuration_type,
            "controllers.yaml",
        ]
    )
    tasks_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            configuration_type,
            "tasks.yaml",
        ]
    )
    planning_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            configuration_type,
            "planning.yaml",
        ]
    )
    mux_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            configuration_type,
            "mux.yaml",
        ]
    )
    joy_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            configuration_type,
            "mux.yaml",
        ]
    )

    args = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "The prefix of the system. This is useful for multi-robot setups."
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
            default_value="false",
            description=(
                "Start the Reach Alpha 5 driver using fake hardware mirroring command"
                " to its states. If this is set to 'false', the Alpha 5 manipulator"
                " serial port should be specified."
            ),
        ),
        DeclareLaunchArgument(
            "alpha_serial_port",
            default_value="''",
            description=(
                "The serial port that the Alpha 5 is available at (e.g., /dev/ttyUSB0)."
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="view_bluerov2_heavy_alpha.rviz",
            description="The RViz2 configuration file.",
        ),
        DeclareLaunchArgument(
            "alpha_controller",
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
            "bluerov2_controller",
            default_value="ismc",
            description=(
                "The controller to use; this should be the same name as the"
                " controller's executable."
            ),
            choices=["ismc"],
        ),
        DeclareLaunchArgument(
            "localization_source",
            default_value="gazebo",
            choices=["mocap", "camera", "gazebo", "hinsdale"],
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
            "use_waypoint_planner",
            default_value="true",
            description="Load a waypoint planner.",
        ),
        DeclareLaunchArgument(
            "use_whole_body_control",
            default_value="true",
            description="Load a whole-body controller.",
        ),
        DeclareLaunchArgument(
            "use_joy", default_value="true", description="Use a joystick controller."
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="bluerov2_heavy_alpha_underwater.world",
            description="The Gazebo world file to launch.",
        ),
        DeclareLaunchArgument(
            "waypoint_planner",
            default_value="preplanned_end_effector_waypoint_planner",
            description="The waypoint planner to load.",
            choices=["preplanned_end_effector_waypoint_planner"],
        ),
        DeclareLaunchArgument(
            "whole_body_controller",
            default_value="tpik_joint_trajectory_controller",
            description="The whole-body controller to load.",
            choices=["tpik_joint_trajectory_controller"],
        ),
    ]

    use_sim = LaunchConfiguration("use_sim")
    namespace = LaunchConfiguration("namespace")
    alpha_controller = LaunchConfiguration("alpha_controller")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
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
            LaunchConfiguration("alpha_serial_port"),
            " ",
            "namespace:=",
            namespace,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[
            controllers_file,
            {
                "use_sim_time": use_sim,
                "robot_description": robot_description,
            },
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

    alpha_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[
            alpha_controller,
            "--controller-manager",
            [namespace, "controller_manager"],
        ],
        parameters=[{"use_sim_time": use_sim}],
    )

    nodes = [
        control_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[alpha_controller_spawner],
            )
        ),
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_bringup"), "launch", "base.launch.py"]
                )
            ),
            launch_arguments={
                "description_package": description_package,
                "configuration_type": "bluerov2_heavy_alpha",
                "controller": LaunchConfiguration("bluerov2_controller"),
                "localization_source": LaunchConfiguration("localization_source"),
                "use_camera": LaunchConfiguration("use_camera"),
                "use_mocap": LaunchConfiguration("use_mocap"),
                "use_sim": LaunchConfiguration("use_sim"),
                "use_rviz": "false",  # Use our own version
                "use_joy": LaunchConfiguration("use_joy"),
                "gazebo_world_file": LaunchConfiguration("gazebo_world_file"),
                "prefix": LaunchConfiguration("prefix"),
                "robot_description": robot_description,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("angler_planning"), "planning.launch.py"]
                )
            ),
            launch_arguments={
                "use_waypoint_planner": LaunchConfiguration("use_waypoint_planner"),
                "waypoint_planner": LaunchConfiguration("waypoint_planner"),
                "config_filepath": planning_file,
                "use_sim_time": use_sim,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("angler_mux"), "mux.launch.py"])
            ),
            launch_arguments={
                "config_filepath": mux_file,
                "use_sim_time": use_sim,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("angler_control"), "control.launch.py"]
                )
            ),
            launch_arguments={
                "controller": LaunchConfiguration("whole_body_controller"),
                "config_filepath": controllers_file,
                "hierarchy_file": tasks_file,
                "use_sim_time": use_sim,
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_whole_body_control")),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution(
        #             [FindPackageShare("angler_behaviors"), "behavior_tree.launch.py"]
        #         )
        #     ),
        #     launch_arguments={"use_sim_time": use_sim}.items(),
        # ),
    ]

    return LaunchDescription(args + nodes + includes)
