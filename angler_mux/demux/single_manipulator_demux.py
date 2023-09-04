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

import rclpy
from geometry_msgs.msg import Twist
from moveit_msgs.msg import RobotTrajectory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class SingleManipulatorVelocityDemux(Node):
    """Demux a RobotTrajectory velocity command to a vehicle and a single manipulator.

    The SingleManipulatorVelocityDemux demuxes the RobotTrajectory into a single vehicle
    velocity Twist command and a single manipulator joint velocity Float64MultiArray
    command.
    """

    def __init__(self) -> None:
        """Create a new demuxer for the vehicle and a manipulator."""
        super().__init__("single_manipulator_velocity_demux")

        self.declare_parameters(
            namespace="",
            parameters=[  # type: ignore
                ("vehicle_control_topic", "/blue/ismc/cmd_vel"),
                ("manipulator_control_topic", "/forward_velocity_controller/commands"),
            ],
        )

        vehicle_control_topic = (
            self.get_parameter("vehicle_control_topic")
            .get_parameter_value()
            .string_value
        )
        manipulator_control_topic = (
            self.get_parameter("manipulator_control_topic")
            .get_parameter_value()
            .string_value
        )

        self.vehicle_cmd_pub = self.create_publisher(Twist, vehicle_control_topic, 1)
        self.alpha_cmd_pub = self.create_publisher(
            Float64MultiArray, manipulator_control_topic, 1
        )

        self.robot_trajectory_sub = self.create_subscription(
            RobotTrajectory, "/angler/robot_trajectory", self.demux_cmd_vel, 1
        )

    def demux_cmd_vel(self, cmd: RobotTrajectory) -> None:
        """Demux the RobotTrajectory into a vehicle cmd and a single manipulator cmd.

        Args:
            cmd: The robot control command.
        """

        # Proxy the vehicle velocity command
        self.vehicle_cmd_pub.publish(
            cmd.multi_dof_joint_trajectory.points[0].velocities[0]  # type: ignore
        )

        # Now proxy the manipulator joint velocity command
        self.alpha_cmd_pub.publish(
            Float64MultiArray(
                data=cmd.joint_trajectory.points[0].velocities  # type: ignore
            )
        )


def main_single_manipulator_velocity_demux(args: list[str] | None = None):
    """Run the mux for a single manipulator."""
    rclpy.init(args=args)

    node = SingleManipulatorVelocityDemux()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
