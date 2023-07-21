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
from geometry_msgs.msg import Transform
from moveit_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class Mux(Node):
    """Base class for an state mux.

    This interface is designed to capture all UVMS state information and proxy it on a
    single topic.
    """

    def __init__(self, node_name: str = "angler_mux") -> None:
        """Create a new mux base object.

        Args:
            node_name: The name of the ROS 2 node. Defaults to "angler_mux".
        """
        super().__init__(node_name)

        # Maintain the UVMS state
        self.uvms_state = RobotState()

        # Publishers
        self.uvms_state_pub = self.create_publisher(RobotState, "/angler/state", 1)

        # Subscribers
        self.base_state_sub = self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.update_base_state_cb,
            qos_profile_sensor_data,
        )

    def update_base_state_cb(self, odom: Odometry) -> None:
        """Update the current AUV state.

        Args:
            odom: The current state (pose + velocity) of the base.
        """
        # Convert the message into a Transform message
        transform = Transform()

        # This is the transform from the map frame to the base_link frame
        transform.translation.x = odom.pose.pose.position.x
        transform.translation.y = odom.pose.pose.position.y
        transform.translation.z = odom.pose.pose.position.z
        transform.rotation = odom.pose.pose.orientation

        # Update the MultiDOFJointState
        self.uvms_state.multi_dof_joint_state.header.frame_id = "map"
        self.uvms_state.multi_dof_joint_state.header.stamp = self.get_clock().now()
        self.uvms_state.multi_dof_joint_state.joint_names = ["vehicle"]
        self.uvms_state.multi_dof_joint_state.transforms = [transform]
        self.uvms_state.multi_dof_joint_state.twist = [odom.twist.twist]

        # Publish the UVMS state each time we get an update
        self.uvms_state_pub.publish(self.uvms_state)


class SingleManipulatorMux(Mux):
    """UVMS state mux for a UVMS with a single Reach Alpha manipulator."""

    def __init__(self) -> None:
        """Create a new mux for a single manipulator."""
        super().__init__(node_name="single_manipulator_mux")

        # Subscribers
        self.manipulator_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.update_joint_state_cb,
            1,
        )

    def update_joint_state_cb(self, state: JointState) -> None:
        """Update the current joint state information.

        Args:
            state: The current Reach Alpha joint state.
        """
        self.uvms_state.joint_state = state

        # Publish the UVMS state each time we get an update
        self.uvms_state_pub.publish(self.uvms_state)


def main_single_manipulator_mux(args: list[str] | None = None):
    """Run the mux for a single manipulator."""
    rclpy.init(args=args)

    node = SingleManipulatorMux()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
