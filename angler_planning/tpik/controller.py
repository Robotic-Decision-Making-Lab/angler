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

import kinpy
import numpy as np
import rclpy
from moveit_msgs.msg import RobotState, RobotTrajectory
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tpik.hierarchy import TaskHierarchy


def calculate_nullspace(augmented_jacobian: np.ndarray) -> np.ndarray:
    """Calculate the nullspace of the augmented Jacobian.

    Args:
        augmented_jacobian: The augmented Jacobian whose nullspace will be projected
            into.

    Returns:
        The nullspace of the augmented Jacobian.
    """
    return (
        np.eye(augmented_jacobian.shape[0])
        - np.linalg.pinv(augmented_jacobian) @ augmented_jacobian
    )


def construct_augmented_jacobian(jacobians: list[np.ndarray]) -> np.ndarray:
    """Construct an augmented Jacobian matrix.

    The augmented Jacobian matrix is given as:

    J_i^A = [J_1, J_2, ..., J_i]^T

    Args:
        jacobians: The Jacobian matrices which should be used to construct the
            augmented Jacobian.

    Returns:
        The resulting augmented Jacobian.
    """
    return np.vstack(tuple(jacobians))


class TPIK(Node):
    """Task-priority inverse kinematic (TPIK) controller.

    The TPIK controller is responsible for calculating kinematically feasible system
    velocities subject to equality and set-based constraints.
    """

    def __init__(self) -> None:
        """Create a new TPIK node."""
        super().__init__("tpik")

        hierarchy = TaskHierarchy()

        hierarchy.load_constraints_from_path(
            "/workspaces/angler/angler_description/config/tasks.yaml"
        )

        self.declare_parameter("robot_description", "")
        self.declare_parameter("manipulator_base_link", "alpha_base_link")
        self.declare_parameter("manipulator_end_link", "alpha_ee_base_link")

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.robot_state_sub = self.create_subscription(
            RobotState, "/angler/state", self.robot_state_cb, 1
        )
        self.joint_trajectory_sub = self.create_subscription(
            RobotTrajectory,
            "/angler/reference_trajectory/",
            self.update_trajectory_cb,
            1,
        )

        # Publishers
        self.robot_trajectory_pub = self.create_publisher(
            RobotTrajectory, "/angler/robot_trajectory", 1
        )

        # Read the robot description to a serial chain
        robot_description = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        end_link = (
            self.get_parameter("manipulator_end_link")
            .get_parameter_value()
            .string_value
        )
        start_link = (
            self.get_parameter("manipulator_base_link")
            .get_parameter_value()
            .string_value
        )

        self.serial_chain = kinpy.build_serial_chain_from_urdf(
            robot_description,
            end_link_name=end_link,
            root_link_name=start_link,
        )

        self.num_manipulator_joints = len(self.serial_chain.get_joint_parameter_names())

    def robot_state_cb(self, state: RobotState) -> None:
        ...

    def update_trajectory_cb(self, trajectory: RobotTrajectory) -> None:
        ...

    def update_context(self) -> None:
        ...

    def update(self) -> None:
        def calculate_system_velocity(
            task: int,
            total_tasks: int,
            nullspace: np.ndarray,
            system_velocities: np.ndarray,
        ):
            # Get the jacobian
            J = np.zeros((6, 6))


def main(args: list[str] | None = None):
    """Run the TPIK controller."""
    rclpy.init(args=args)

    node = TPIK()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
