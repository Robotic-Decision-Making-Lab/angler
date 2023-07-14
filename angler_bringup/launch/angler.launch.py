# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            choices=["forward_velocity_controller", "forward_position_controller"],
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
            "use_rviz", default_value="true", description="Launch RViz2."
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    manipulator_controller = LaunchConfiguration("manipulator_controller")
    use_rviz = LaunchConfiguration("use_rviz")
    controllers_file = LaunchConfiguration("controllers_file")

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_manipulator_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            manipulator_controller,
        ],
        output="screen",
    )

    nodes = [
        gz_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawner,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_manipulator_controller],
            )
        ),
        Node(
            package="mavros",
            executable="mavros_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("mavros_file"),
                    ]
                )
            ],
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
                    [
                        FindPackageShare(description_package),
                        "launch",
                        "description.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "use_rviz": use_rviz,
                "controllers_file": controllers_file,
            }.items(),
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
                )
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
            }.items(),
        ),
    ]

    return LaunchDescription(args + nodes + includes)
