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
from geometry_msgs.msg import Transform, Twist
from moveit_msgs.msg import RobotState, RobotTrajectory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import TransformException  # type: ignore
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tpik.hierarchy import TaskHierarchy
from tpik.tasks import (
    EndEffectorPose,
    JointLimit,
    ManipulatorConfiguration,
    VehicleOrientation,
    VehicleRollPitch,
    VehicleYaw,
)
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint


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

        self.declare_parameter("constraint_config_path", "")
        self.declare_parameter("manipulator_base_link", "alpha_base_link")
        self.declare_parameter("manipulator_end_link", "alpha_ee_base_link")

        # Keep track of the robot state for the tasks
        self.state = RobotState()

        # Get the constraints
        self.hierarchy = TaskHierarchy.load_constraints_from_path(
            self.get_parameter("constraint_config_path")
            .get_parameter_value()
            .string_value
        )

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.robot_description_sub = self.create_subscription(
            String,
            "/robot_description",
            self.read_robot_description_cb,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.robot_state_sub = self.create_subscription(
            RobotState, "/angler/state", self.robot_state_cb, 1
        )
        self.joint_trajectory_sub = self.create_subscription(
            RobotTrajectory,
            "/angler/reference_trajectory",
            self.update_trajectory_cb,
            1,
        )

        # Publishers
        self.robot_trajectory_pub = self.create_publisher(
            RobotTrajectory, "/angler/robot_trajectory", 1
        )

    def read_robot_description_cb(self, robot_description: String) -> None:
        """Create a kinpy serial chain from the robot description.

        Args:
            robot_description: The robot description message.
        """
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
            robot_description.data,
            end_link_name=end_link,
            root_link_name=start_link,
        )

        self.n_manipulator_joints = len(self.serial_chain.get_joint_parameter_names())

        # Set the relevant task properties while we are here so that we don't have to
        # do it later
        for task in self.hierarchy.constraints:
            if hasattr(task, "n_manipulator_joints"):
                task.n_manipulator_joints = self.n_manipulator_joints  # type: ignore

            if hasattr(task, "serial_chain"):
                task.serial_chain = self.serial_chain  # type: ignore

    def robot_state_cb(self, state: RobotState) -> None:
        """Update the current robot state.

        Args:
            state: The current robot state.
        """
        self.state = state

        # Update the task context
        self.update_context()

        # Now update the current task hierarchy
        # TODO
        ...

    def update_context(self) -> None:
        """Update the current state variables for each task."""
        for task in self.hierarchy.activated_task_hierarchy:
            # As a brief note, the joint state includes the linear jaws joint angle.
            # We exclude this, because we aren't controlling it within this specific
            # framework.
            if isinstance(task, JointLimit):
                # Joint limits appear the most frequently so update those first
                joint_angles = np.array(self.state.joint_state.position)[1:]  # type: ignore # noqa
                task.update(joint_angles[task.joint])
            elif isinstance(task, VehicleRollPitch):
                vehicle_pose: Transform = self.state.multi_dof_joint_state.transforms[0]  # type: ignore # noqa
                task.update(vehicle_pose.rotation)
            elif isinstance(task, ManipulatorConfiguration):
                joint_angles = np.array(self.state.joint_state.position)[1:]  # type: ignore # noqa
                task.update(joint_angles)
            elif isinstance(task, VehicleYaw):
                vehicle_pose: Transform = self.state.multi_dof_joint_state.transforms[0]  # type: ignore # noqa
                task.update(vehicle_pose.rotation)
            elif isinstance(task, VehicleOrientation):
                vehicle_pose: Transform = self.state.multi_dof_joint_state.transforms[0]  # type: ignore # noqa
                task.update(vehicle_pose.rotation)
            elif isinstance(task, EndEffectorPose):
                joint_angles = np.array(self.state.joint_state.position)[1:]  # type: ignore # noqa
                vehicle_pose: Transform = self.state.multi_dof_joint_state.transforms[0]  # type: ignore # noqa

                # Get the necessary transforms
                try:
                    tf_manipulator_base_to_base = self.tf_buffer.lookup_transform(
                        "alpha_base_link", "base_link", self.get_clock().now()
                    )
                except TransformException as e:
                    self.get_logger().error(
                        "Unable to get the transformation from the manipulator base"
                        f" to the vehicle base: {e}"
                    )
                    continue
                try:
                    tf_map_to_ee = self.tf_buffer.lookup_transform(
                        "map", "alpha_ee_base_link", self.get_clock().now()
                    )
                except TransformException as e:
                    self.get_logger().error(
                        "Unable to get the transformation from the manipulator base"
                        f" to the vehicle base: {e}"
                    )
                    continue

                task.update(
                    joint_angles,
                    vehicle_pose,
                    tf_manipulator_base_to_base.transform,
                    tf_map_to_ee.transform,
                )

    def update_trajectory_cb(self, trajectory: RobotTrajectory) -> None:
        ...

    def calculate_system_velocity(
        self,
        task: int = 0,
        system_velocities: np.ndarray | None = None,
        prev_jacobians: list[np.ndarray] | None = None,
        nullspace: np.ndarray | None = None,
    ):
        """Calculate the system velocities using TPIK control.

        This method calculates the system velocities recursively. This is based off of
        the formula provided in "Underwater Intervention With Remote Supervision via
        Satellite Communication: Developed Control Architecture and Experimental Results
        Within the Dexrov Project" by Di Lillo, et. al.

        Args:
            task: The current task index in the activated task hierarchy. Defaults to 0.
            system_velocities: The calculated system velocities. Defaults to None.
            prev_jacobians: The list of previous Jacobians. Defaults to None.
            nullspace: The nullspace which the task Jacobian should be projected into.
                Defaults to None.

        Returns:
            The calculated system velocities.
        """
        # Get the task
        t = self.hierarchy.activated_task_hierarchy[task]

        J = t.jacobian
        K = t.gain
        r = t.error

        # Calculate the system velocities
        if task == 0 or None in [system_velocities, prev_jacobians, nullspace]:
            # Set the initial system velocities to 0
            system_velocities = np.zeros((6 + self.n_manipulator_joints, 1))

            if prev_jacobians is None:
                prev_jacobians = []

            # The nullspace for the first task is the identity matrix
            updated_system_velocities = (
                np.linalg.pinv(J) * K @ r - J * system_velocities
            )
        else:
            updated_system_velocities = (
                np.linalg.pinv(J @ nullspace) * K @ r - J * system_velocities
            )

        # Stop recursion
        if task == len(self.hierarchy.activated_task_hierarchy) - 1:
            return updated_system_velocities

        # Now update for the next iteration
        # Note that this will modify the top-level list
        prev_jacobians.append(J)  # type: ignore
        updated_nullspace = calculate_nullspace(
            construct_augmented_jacobian(prev_jacobians)  # type: ignore
        )

        return self.calculate_system_velocity(
            task=task + 1,
            prev_jacobians=prev_jacobians,
            system_velocities=updated_system_velocities,
            nullspace=updated_nullspace,
        )

    def update(self) -> None:
        """Send the updated system velocities."""
        # Update all context variables are updated prior to running the control loop
        self.update_context()

        # Calculate the system velocities
        system_velocities = self.calculate_system_velocity(
            len(self.hierarchy.activated_task_hierarchy)
        )

        cmd = RobotTrajectory()

        # Create the vehicle command
        vehicle_cmd = MultiDOFJointTrajectoryPoint()
        vehicle_vel = Twist()

        (
            vehicle_vel.linear.x,
            vehicle_vel.linear.y,
            vehicle_vel.linear.z,
            vehicle_vel.angular.x,
            vehicle_vel.angular.y,
            vehicle_vel.angular.z,
        ) = system_velocities[:6]

        vehicle_cmd.velocities = [vehicle_vel]

        # Create the manipulator command
        arm_cmd = JointTrajectoryPoint()
        arm_cmd.velocities = [0.0] + list(system_velocities[6:])

        # Create the full system command
        cmd.multi_dof_joint_trajectory.joint_names = ["vehicle"]
        cmd.joint_trajectory.joint_names = self.serial_chain.get_joint_parameter_names()
        cmd.multi_dof_joint_trajectory.points = [vehicle_cmd]
        cmd.joint_trajectory.points = [arm_cmd]

        self.robot_trajectory_pub.publish(cmd)


def main(args: list[str] | None = None):
    """Run the TPIK controller."""
    rclpy.init(args=args)

    node = TPIK()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
