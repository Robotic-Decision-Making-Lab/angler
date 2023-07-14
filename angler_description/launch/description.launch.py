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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
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
            description="The URDF/XACRO description file with the Alpha.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="angler_controllers.yaml",
            description="The BlueROV2 Heavy controller configuration file.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="view_angler.rviz",
            description="The RViz2 configuration file.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", description="Launch RViz2."
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
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo controller interface.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    use_sim = LaunchConfiguration("use_sim")

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
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
                LaunchConfiguration("controllers_file"),
                " ",
                "initial_positions_file:=",
                LaunchConfiguration("initial_positions_file"),
                " ",
                "namespace:=",
                LaunchConfiguration("namespace"),
            ]
        )
    }

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
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
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]

    return LaunchDescription(args + nodes)
