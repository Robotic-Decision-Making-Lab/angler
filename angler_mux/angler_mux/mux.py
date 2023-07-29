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
from geometry_msgs.msg import Transform
from message_filters import ApproximateTimeSynchronizer, Subscriber
from moveit_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState


class Mux(Node):
    """Mux all UVMS state information and proxy state to a single topic."""

    def __init__(self, node_name: str = "angler_mux") -> None:
        """Create a new mux interface.

        Args:
            node_name: The name of the ROS 2 node. Defaults to "angler_mux".
        """
        super().__init__(node_name)

        # Maintain the UVMS state
        self.robot_state = RobotState()

        # Publishers
        self.uvms_state_pub = self.create_publisher(
            RobotState, "/angler/state", qos_profile=qos_profile_sensor_data
        )

        # Subscribers
        self.base_state_sub = Subscriber(
            self,
            Odometry,
            "/mavros/local_position/odom",
            qos_profile=qos_profile_sensor_data,
        )
        self.manipulator_state_sub = Subscriber(
            self,
            JointState,
            "/joint_states",
            qos_profile=QoSProfile(
                depth=5, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        # Create a message filter to synchronize state messages
        self.ts = ApproximateTimeSynchronizer(
            [self.base_state_sub, self.manipulator_state_sub], 1, 0.05
        )

        self.ts.registerCallback(self.update_robot_state_cb)

    def update_robot_state_cb(self, odom: Odometry, joint_state: JointState) -> None:
        """Update the current robot state.

        Args:
            odom: The current state (pose + velocity) of the base.
            joint_state: The current joint state of the manipulator(s).
        """
        # Convert the message into a Transform message
        transform = Transform()

        # This is the transform from the map frame to the base_link frame
        transform.translation.x = odom.pose.pose.position.x
        transform.translation.y = odom.pose.pose.position.y
        transform.translation.z = odom.pose.pose.position.z
        transform.rotation = odom.pose.pose.orientation

        # Update the MultiDOFJointState
        self.robot_state.multi_dof_joint_state.header.frame_id = "map"
        self.robot_state.multi_dof_joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        self.robot_state.multi_dof_joint_state.joint_names = ["vehicle"]
        self.robot_state.multi_dof_joint_state.transforms = [transform]
        self.robot_state.multi_dof_joint_state.twist = [odom.twist.twist]

        # Update the joint states
        self.robot_state.joint_state = joint_state

        # Publish the UVMS state each time we get an update
        self.uvms_state_pub.publish(self.robot_state)


def main(args: list[str] | None = None):
    """Run the mux for a single manipulator."""
    rclpy.init(args=args)

    node = Mux()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
