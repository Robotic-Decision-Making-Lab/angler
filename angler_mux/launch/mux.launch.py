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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for the state mux/command demux.

    Returns:
        LaunchDescription: The state mux/command demux launch description.
    """
    args = [
        DeclareLaunchArgument(
            "config_filepath",
            default_value=None,
            description="The path to the configuration YAML file",
        ),
        DeclareLaunchArgument(
            "demux",
            default_value="single_manipulator_velocity_demux",
            description="The command demux type to load.",
            choices=["single_manipulator_velocity_demux"],
        ),
        DeclareLaunchArgument(
            "mux",
            default_value="single_manipulator_velocity_mux",
            description="The state mux type to load.",
            choices=["single_manipulator_velocity_mux"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=("Use the simulated Gazebo clock."),
        ),
    ]

    demux = LaunchConfiguration("demux")
    mux = LaunchConfiguration("mux")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nodes = [
        Node(
            package="angler_mux",
            executable=demux,
            name=demux,
            output="screen",
            parameters=[
                LaunchConfiguration("config_filepath"),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="angler_mux",
            executable=mux,
            name=mux,
            parameters=[
                LaunchConfiguration("config_filepath"),
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]

    return LaunchDescription(args + nodes)
