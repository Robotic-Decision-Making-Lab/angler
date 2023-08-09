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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for the Angler control interface.

    Returns:
        LaunchDescription: The Angler control launch description.
    """
    args = [
        DeclareLaunchArgument(
            "config_filepath",
            default_value=None,
            description="The path to the configuration YAML file.",
        ),
        DeclareLaunchArgument(
            "hierarchy_file",
            default_value=None,
            description="The path to the TPIK task hierarchy.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=("Use the simulated Gazebo clock."),
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="tpik_controller",
            description="The whole-body controller to load.",
            choices=["tpik_controller"],
        ),
    ]

    controller = LaunchConfiguration("controller")

    nodes = [
        # Create a separate node for TPIK because it requires the task hierarchy file
        Node(
            package="angler_control",
            executable=controller,
            name=controller,
            output="screen",
            parameters=[
                {
                    "config_filepath": LaunchConfiguration("config_filepath"),
                    "hierarchy_file": LaunchConfiguration("hierarchy_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            condition=IfCondition(PythonExpression(["'", controller, "' == 'tpik'"])),
        ),
    ]

    return LaunchDescription(args + nodes)
