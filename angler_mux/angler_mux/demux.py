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
from moveit_msgs.msg import RobotTrajectory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint


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
            RobotTrajectory, "/angler/robot_trajectory", self.proxy_cmd_vel_cb, 1
        )

    def proxy_cmd_vel_cb(self, cmd: RobotTrajectory) -> None:
        """Proxy the base velocity command to its respective topic.

        NOTE: This should be overridden to support one or more manipulators.

        Args:
            cmd: The system velocity command.
        """
        try:
            point: MultiDOFJointTrajectoryPoint = (
                cmd.multi_dof_joint_trajectory.points[  # type:ignore
                    0
                ]
            )
            cmd_vel: Twist = point.velocities[0]  # type: ignore
            self.base_cmd_vel_pub.publish(cmd_vel)
        except IndexError as e:
            self.get_logger().error(
                f"An error occurred while attempting to demux the RobotTrajectory: {e}",
            )


class SingleManipulatorForwardVelocityDemux(VelocityDemux):
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
            Float64MultiArray,
            self.get_parameter("manipulator_cmd_vel_topic")
            .get_parameter_value()
            .string_value,
            1,
        )

    def proxy_cmd_vel_cb(self, cmd: RobotTrajectory) -> None:
        """Proxy the manipulator joint velocities.

        Args:
            cmd: The system velocity command.
        """
        # Proxy the base velocity command
        super().proxy_cmd_vel_cb(cmd)

        # Convert the JointTrajectory into a command that the forward velocity
        # controller can use
        joint_vel_cmd = Float64MultiArray()

        point: JointTrajectoryPoint = cmd.joint_trajectory.points[0]  # type:ignore
        joint_vel_cmd.data = point.positions

        # Now proxy the manipulator joint velocity command
        self.manipulator_cmd_vel_pub.publish(joint_vel_cmd)


def main_single_manipulator_demux(args: list[str] | None = None):
    """Run the mux for a single manipulator."""
    rclpy.init(args=args)

    node = SingleManipulatorForwardVelocityDemux()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
