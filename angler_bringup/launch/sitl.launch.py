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
)
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
    ]

    nodes = [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                "angler",
                "-topic",
                "robot_description",
            ],
            output="screen",
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
                        FindPackageShare(LaunchConfiguration("description_package")),
                        "ardusub",
                        LaunchConfiguration("ardusub_params_file"),
                    ]
                ),
                "-I0",
                "--home",
                "44.65870,-124.06556,0.0,270.0",  # my not-so-secret surf spot
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(args + nodes + includes + processes)
