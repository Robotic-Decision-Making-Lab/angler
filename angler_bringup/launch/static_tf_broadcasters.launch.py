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
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for the static TF broadcasters."""
    # The robot_state_broadcaster doesn't get any state information regarding the
    # thruster joint states, so we publish a static transformation to improve the
    # visualizations in RViz2.
    thrusters = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster1",
            arguments=[
                "--x",
                "0.088",
                "--y",
                "-0.102",
                "--z",
                "-0.04",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "-1.047",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster1",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster2",
            arguments=[
                "--x",
                "0.088",
                "--y",
                "0.102",
                "--z",
                "-0.04",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "-2.094",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster2",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster3",
            arguments=[
                "--x",
                "-0.088",
                "--y",
                "-0.102",
                "--z",
                "-0.04",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "1.047",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster3",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster4",
            arguments=[
                "--x",
                "-0.088",
                "--y",
                "0.102",
                "--z",
                "-0.04",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "2.094",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster4",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster5",
            arguments=[
                "--x",
                "0.118",
                "--y",
                "-0.215",
                "--z",
                "0.064",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster5",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster6",
            arguments=[
                "--x",
                "0.118",
                "--y",
                "0.215",
                "--z",
                "0.064",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster6",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster7",
            arguments=[
                "--x",
                "-0.118",
                "--y",
                "-0.215",
                "--z",
                "0.064",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster7",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster8",
            arguments=[
                "--x",
                "-0.118",
                "--y",
                "0.215",
                "--z",
                "0.064",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "thruster8",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_base_footprint",
            arguments=[
                "--x",
                "-0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "base_footprint",
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(thrusters)
