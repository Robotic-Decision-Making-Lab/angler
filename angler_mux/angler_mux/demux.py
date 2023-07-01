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

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from angler_msgs.msg import UvmsCmd


class VelocityDemux(Node):
    """Base class used to demux system velocity commands.

    The Demux base class is designed to receive a single system velocity command, demux
    the command into sub-system velocities, and send the sub-system velocities to their
    respective command interfaces.
    """

    def __init__(self, node_name: str = "angler_demux") -> None:
        """Create a new demux base object.

        Args:
            node_name: The name of the ROS 2 node. Defaults to "angler_demux".
        """
        super().__init__(node_name)

        # Parameters
        self.declare_parameter("base_cmd_vel_topic", "/blue/ismc/cmd_vel")

        # Publishers
        self.base_cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("base_cmd_vel_topic").get_parameter_value().string_value,
            1,
        )

        # Subscribers
        self.uvms_cmd_vel_sub = self.create_subscription(
            UvmsCmd, "/angler/cmd_vel", self.proxy_cmd_vel_cb, 1
        )

    def proxy_cmd_vel_cb(self, cmd: UvmsCmd) -> None:
        """Proxy the base velocity command to its respective topic.

        NOTE: This should be overridden to support one or more manipulators.

        Args:
            cmd: The system velocity command.
        """
        self.base_cmd_vel_pub.publish(cmd.base)


class SingleManipulatorVelocityDemux(VelocityDemux):
    """Demux velocities for a single manipulator with a forward velocity controller."""

    def __init__(self) -> None:
        """Create a new single manipulator demuxer."""
        super().__init__("single_manipulator_demux")

        # Parameters
        self.declare_parameter(
            "manipulator_cmd_vel_topic", "/forward_velocity_controller/commands"
        )

        # Publishers
        self.manipulator_cmd_vel_pub = self.create_publisher(
            JointTrajectoryPoint,
            self.get_parameter("manipulator_cmd_vel_topic")
            .get_parameter_value()
            .string_value,
            1,
        )

    def proxy_cmd_vel_cb(self, cmd: UvmsCmd) -> None:
        """Proxy the manipulator joint velocities.

        Args:
            cmd: The system velocity command.
        """
        # Proxy the base velocity command
        super().proxy_cmd_vel_cb(cmd)

        # Now proxy the manipulator joint velocity command
        self.manipulator_cmd_vel_pub.publish(cmd.manipulator.data)


def main_single_manipulator_demux(args: list[str] | None = None):
    """Run the mux for a single manipulator."""
    rclpy.init(args=args)

    node = SingleManipulatorVelocityDemux()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
