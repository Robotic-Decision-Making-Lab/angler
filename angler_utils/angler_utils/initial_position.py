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

import os

import rclpy
import yaml  # type: ignore
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class InitialPositionSetter(Node):
    """Interface used to set the initial positions of joints.

    WARNING: This is a temporary hack used specifically for when a velocity controller
    has been loaded but the initial position is not properly set in Gazebo. See
    https://github.com/ros-controls/gz_ros2_control/issues/167 for more information. Yes
    there are probably plenty of bugs in this, but really I just need it to work well
    enough until there is a real fix implemented.
    """

    def __init__(self) -> None:
        """Create a new initial position setter."""
        super().__init__("initial_position_setter")

        self.declare_parameters(
            namespace="",
            parameters=[  # type: ignore
                (
                    "initial_positions_file",
                    os.path.join(
                        get_package_share_directory("angler_description"),
                        "config",
                        "initial_positions.yaml",
                    ),
                ),
                ("controller_cmd_topic", "/forward_velocity_controller/commands"),
                ("position_tol", 0.1),
                ("joint_velocity", 0.5),
            ],
        )

        # Get the desired initial positions
        with open(
            self.get_parameter("initial_positions_file")
            .get_parameter_value()
            .string_value,
            "r",
        ) as file:
            self.desired_initial_positions_dict = yaml.safe_load(file)[
                "initial_positions"
            ]

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("controller_cmd_topic")
            .get_parameter_value()
            .string_value,
            1,
        )

        # Keep track of the initial states so that we know when to stop
        self.actual_initial_positions: list[float] | None = None
        self.set_initial_positions = True

        self.get_logger().info(
            "Attempting to drive the manipulator to the desired initial positions."
        )

        # Configure how fast to go to the initial positions and when to stop
        self.tol = self.get_parameter("position_tol").get_parameter_value().double_value
        self.joint_velocity = (
            self.get_parameter("joint_velocity").get_parameter_value().double_value
        )

        # Subscribers
        self.create_subscription(
            JointState, "/joint_states", self.set_initial_positions_cb, 1
        )

    @staticmethod
    def construct_velocity_command(
        distances: list[float], tol: float, joint_velocity: float
    ) -> Float64MultiArray:
        """Create a velocity command to drive the manipulator to the initial positions.

        Args:
            distances: The distances between the desired initial positions and the
                actual initial positions.
            tol: The minimum distance between the actual joint position and desired
                joint position before the system stops sending commands.
            joint_velocity: The velocity that the joints should move at while going to
                the desired positions.

        Returns:
            A joint velocity command to drive the robot to the desired initial
            positions.
        """
        cmd = Float64MultiArray()

        for distance in distances:
            if distance < tol:
                cmd.data.append(0.0)
            else:
                if distance < 0:
                    cmd.data.append(-1 * abs(joint_velocity))
                else:
                    cmd.data.append(abs(joint_velocity))

        return cmd

    def set_initial_positions_cb(self, msg: JointState) -> None:
        """Drive the manipulator to the desired initial positions.

        Args:
            msg: The current joint state.
        """
        # Break early
        if not self.set_initial_positions:
            return

        desired_initial_positions = []

        # Get the initial positions as a list
        # Make sure that each of the joints exists while we are at it
        for joint_name in msg.name:
            try:
                desired_initial_positions.append(
                    self.desired_initial_positions_dict[joint_name]
                )
            except KeyError as e:
                self.get_logger().error(
                    "Invalid mapping between the desired joint state joint names and"
                    " the actual joint names. Desired initial position joint names:"
                    f" {list(self.desired_initial_positions_dict.keys())}; actual joint"
                    f" names: {msg.name}. {e}"
                )
                self.set_initial_positions = False

        # Get the errors
        distances = [
            desired - actual
            for (desired, actual) in zip(
                desired_initial_positions, msg.position  # type: ignore
            )
        ]

        if any([error > self.tol for error in distances]):
            cmd = self.construct_velocity_command(
                distances, self.tol, self.joint_velocity
            )
            self.cmd_vel_pub.publish(cmd)
        else:
            self.set_initial_positions = False

            # Stop the arm
            cmd = Float64MultiArray()
            cmd.data = [0.0 for _ in range(len(distances))]
            self.cmd_vel_pub.publish(cmd)

            self.get_logger().info("Successfully set the initial joint positions.")


def main(args: list[str] | None = None):
    """Run the initial position setter."""
    rclpy.init(args=args)

    node = InitialPositionSetter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
