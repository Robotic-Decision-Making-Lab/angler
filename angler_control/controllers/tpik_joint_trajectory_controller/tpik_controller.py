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
from controllers.robot_trajectory_controller.base_multidof_joint_trajectory_controller import (  # noqa
    BaseMultiDOFJointTrajectoryController,
)
from controllers.tpik_joint_trajectory_controller.constraints import (
    EqualityConstraint,
    SetConstraint,
)
from controllers.tpik_joint_trajectory_controller.hierarchy import TaskHierarchy
from controllers.tpik_joint_trajectory_controller.tasks import (
    EndEffectorPoseTask,
    ManipulatorJointConfigurationTask,
    ManipulatorJointLimitTask,
    VehicleRollPitchTask,
    VehicleYawTask,
)
from geometry_msgs.msg import Transform, Twist
from moveit_msgs.msg import RobotState, RobotTrajectory
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import TransformException  # type: ignore
from tf2_ros import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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
        np.eye(augmented_jacobian.shape[1])
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


class TpikController(BaseMultiDOFJointTrajectoryController):
    """Set-based task-priority inverse kinematic (TPIK) controller.

    The TPIK controller is responsible for calculating kinematically feasible system
    velocities subject to equality and set-based constraints.

    Only high-priority set-based tasks have been implemented at the moment.
    """

    def __init__(self) -> None:
        """Create a new TPIK node."""
        super().__init__("tpik_joint_trajectory_controller")

        self.declare_parameter("hierarchy_file", "")
        self.declare_parameters(
            "frames",
            parameters=[  # type: ignore
                ("inertial_frame", "map"),
                ("base_frame", "base_link"),
                ("manipulator_base_link", "alpha_base_link"),
                ("manipulator_end_link", "alpha_standard_jaws_tool"),
            ],
        )

        # Don't run the controller until everything has been properly initialized
        self._description_received = False
        self._init_state_received = False

        # Keep track of the frames
        self.inertial_frame = (
            self.get_parameter("frames.inertial_frame")
            .get_parameter_value()
            .string_value
        )
        self.base_frame = (
            self.get_parameter("frames.base_frame").get_parameter_value().string_value
        )
        self.manipulator_base_frame = (
            self.get_parameter("frames.manipulator_base_link")
            .get_parameter_value()
            .string_value
        )
        self.manipulator_ee_frame = (
            self.get_parameter("frames.manipulator_end_link")
            .get_parameter_value()
            .string_value
        )

        # Get the constraints
        self.hierarchy = TaskHierarchy.load_tasks_from_path(
            self.get_parameter("hierarchy_file").get_parameter_value().string_value
        )

        # TF2
        self.tf_buffer = Buffer(cache_time=Duration(seconds=1))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.robot_description_sub = self.create_subscription(
            String,
            "/robot_description",
            self.read_robot_description_cb,
            QoSProfile(depth=5, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        # Publishers
        self.robot_trajectory_pub = self.create_publisher(
            RobotTrajectory, "/angler/robot_trajectory", 1
        )

    @property
    def joint_state(self) -> MultiDOFJointTrajectoryPoint:
        """Get the end-effector pose in the inertial frame.

        We control the end-effector state with this particular variation of the
        controller.

        Returns:
            The end-effector pose in the inertial frame.
        """
        point = MultiDOFJointTrajectoryPoint()

        # Control the end-effector pose
        try:
            tf_map_to_ee = self.tf_buffer.lookup_transform(
                self.inertial_frame,
                self.manipulator_ee_frame,
                Time(),
                Duration(nanoseconds=10000000),  # 10 ms
            )
        except TransformException as e:
            self.get_logger().warning(
                "Failed to get the current transformation from the map to end-effector"
                f"frame: {e}"
            )
            return point

        point.transforms.append(tf_map_to_ee.transform)  # type: ignore
        return point

    def on_arm(self) -> bool:
        """Make sure that the system has been initialized before enabling arming.

        The controller is considered initialized when the robot description and initial
        state have been received.

        Returns:
            Whether or not the system has been fully initialized.
        """
        return self._description_received and self._init_state_received

    def read_robot_description_cb(self, robot_description: String) -> None:
        """Create a kinpy serial chain from the robot description.

        Args:
            robot_description: The robot description message.
        """
        end_link = (
            self.get_parameter("frames.manipulator_end_link")
            .get_parameter_value()
            .string_value
        )
        start_link = (
            self.get_parameter("frames.manipulator_base_link")
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
        for task in self.hierarchy.tasks:
            if hasattr(task, "n_manipulator_joints"):
                task.n_manipulator_joints = self.n_manipulator_joints  # type: ignore

            if hasattr(task, "serial_chain"):
                task.serial_chain = self.serial_chain  # type: ignore

        self._description_received = True
        self.get_logger().debug("Robot description received!")

    def calculate_system_velocity(
        self, hierarchy: list[EqualityConstraint | SetConstraint]
    ) -> np.ndarray:
        """Calculate the system velocities using TPIK control.

        Args:
            hierarchy: The task hierarchy to use when calculating the system velocities.
        """
        # Set the initial recursion variables
        system_velocities = np.zeros((6 + self.n_manipulator_joints, 1))
        jacobians: list[np.ndarray] = []

        # The initial nullspace matrix is the identity matrix
        # make sure that we use the correct dimensions for it
        nullspace = np.eye(hierarchy[0].jacobian.shape[1])

        def calculate_system_velocity_rec(
            hierarchy: list[EqualityConstraint | SetConstraint],
            system_velocities: np.ndarray,
            jacobians: list[np.ndarray],
            nullspace: np.ndarray,
        ) -> np.ndarray:
            if not hierarchy:
                return system_velocities
            else:
                t = hierarchy[0]
                J = t.jacobian
                e = t.error
                K = t.gain * np.eye(e.shape[0])

                # Use the desired time derivative if the task is time-varying,
                # otherwise use a regularization task
                if t.desired_value_dot is not None:
                    t_dot = t.desired_value_dot
                else:
                    t_dot = np.zeros(e.shape)

                updated_system_velocities = np.linalg.pinv(J @ nullspace) @ (
                    t_dot + K @ e - J @ system_velocities
                )

                jacobians.append(J)  # type: ignore
                updated_nullspace = calculate_nullspace(
                    construct_augmented_jacobian(jacobians)  # type: ignore
                )

                return updated_system_velocities + calculate_system_velocity_rec(
                    hierarchy[1:],
                    updated_system_velocities,
                    jacobians,
                    updated_nullspace,
                )

        return calculate_system_velocity_rec(
            hierarchy, system_velocities, jacobians, nullspace
        )

    def on_update(self) -> None:
        """Calculate the desired system velocities using set-based TPIK."""
        # Update the joint command first
        super().on_update()

        hierarchies = self.hierarchy.hierarchies

        # Set the desired trajectory value
        for hierarchy in hierarchies:
            for task in hierarchy:
                if isinstance(task, EndEffectorPoseTask) and self.command is not None:
                    task.desired_value = self.command.transforms[0]  # type: ignore

        # Check whether or not the hierarchies have any set tasks
        has_set_tasks = any(
            [any([isinstance(y, SetConstraint) for y in x]) for x in hierarchies]
        )

        if not has_set_tasks:
            # If there are only equality tasks in the hierarchies, then there will
            # only be one potential solution
            system_velocities = self.calculate_system_velocity(hierarchies[0])
        else:
            # Otherwise, we need to check all potential solutions to find the best
            solutions = []

            for hierarchy in hierarchies:
                solution = self.calculate_system_velocity(hierarchy)

                set_tasks = [
                    set_task
                    for set_task in self.hierarchy.active_task_hierarchy
                    if isinstance(set_task, SetConstraint)
                ]

                # Check whether or not the solution will drive the system to the safe
                # set, this should always have one solution (all set tasks are
                # activated)
                satisfied = []
                for set_task in set_tasks:
                    projection = set_task.jacobian @ solution

                    if (
                        set_task.current_value < set_task.activation_threshold.lower
                        and projection > 0
                    ):
                        satisfied.append(True)
                    elif (
                        set_task.current_value > set_task.activation_threshold.upper
                        and projection < 0
                    ):
                        satisfied.append(True)
                    elif np.isclose(projection, 0.0, atol=0.02):  # 1 degree
                        satisfied.append(True)
                    else:
                        satisfied.append(False)

                if all(satisfied):
                    solutions.append(solution)

            # Select the solution with the highest norm (this is the least conservative
            # solution)
            try:
                system_velocities = solutions[
                    np.argmax([np.linalg.norm(x) for x in solutions])
                ]
            except Exception as e:
                self.get_logger().debug(
                    "Unable to calculate valid system velocities from the current"
                    f" hierarchy: {e}"
                )
                return

        self.robot_trajectory_pub.publish(
            self.get_robot_trajectory_from_velocities(system_velocities)
        )

    def get_robot_trajectory_from_velocities(
        self, system_velocities: np.ndarray
    ) -> RobotTrajectory:
        """Create a RobotTrajectory message from the system velocities.

        Args:
            system_velocities: The desired system velocites.

        Returns:
            The resulting RobotTrajectory message.
        """
        trajectory = RobotTrajectory()

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
        ) = system_velocities[:6, 0]

        vehicle_cmd.velocities = [vehicle_vel]

        # Create the manipulator command
        arm_cmd = JointTrajectoryPoint()
        arm_cmd.velocities = [0.0] + list(system_velocities[6:, 0])[::-1]

        # Create the full system command
        trajectory.multi_dof_joint_trajectory.joint_names = ["vehicle"]
        # TODO(evan): don't hard-code this
        trajectory.joint_trajectory.joint_names = [
            "alpha_axis_a"
        ] + self.serial_chain.get_joint_parameter_names()[::-1]
        trajectory.multi_dof_joint_trajectory.points = [vehicle_cmd]
        trajectory.joint_trajectory.points = [arm_cmd]

        return trajectory

    def on_robot_state_update(self, state: RobotState) -> None:
        """Update the current task contexts.

        Args:
            state: The current state of the robot.
        """
        # Indicate that the initial robot state has been received
        if not self._init_state_received:
            self._init_state_received = True

        # Get some of the more commonly used state variables
        # The joint state includes the linear jaws joint angle. We exclude this,
        # because we aren't controlling it within this specific framework.
        joint_angles = np.array(state.joint_state.position[1:][::-1])
        vehicle_pose: Transform = state.multi_dof_joint_state.transforms[0]  # type: ignore # noqa

        for task in self.hierarchy.tasks:
            if isinstance(task, ManipulatorJointLimitTask):
                joint_angle = joint_angles[task.joint - 6]
                task.update(joint_angle)
                task.set_task_active(joint_angle)
            elif isinstance(task, VehicleRollPitchTask):
                task.update(vehicle_pose.rotation)
            elif isinstance(task, ManipulatorJointConfigurationTask):
                task.update(joint_angles.reshape((len(joint_angles), 1)))
            elif isinstance(task, VehicleYawTask):
                task.update(vehicle_pose.rotation)
            elif isinstance(task, EndEffectorPoseTask):
                # Get the necessary transforms
                try:
                    tf_map_to_ee = self.tf_buffer.lookup_transform(
                        self.inertial_frame,
                        self.manipulator_ee_frame,
                        Time(),
                        Duration(nanoseconds=10000000),  # 10 ms
                    )
                    tf_base_to_manipulator_base = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.manipulator_base_frame,
                        Time(),
                        Duration(nanoseconds=10000000),  # 10 ms
                    )
                    tf_manipulator_base_to_ee = self.tf_buffer.lookup_transform(
                        self.manipulator_base_frame,
                        self.manipulator_ee_frame,
                        Time(),
                        Duration(nanoseconds=10000000),  # 10 ms
                    )
                except TransformException as e:
                    self.get_logger().error(
                        f"Unable to update the end-effector pose task: {e}"
                    )
                    continue

                task.update(
                    joint_angles,
                    tf_map_to_ee.transform,
                    vehicle_pose,
                    tf_base_to_manipulator_base.transform,
                    tf_manipulator_base_to_ee.transform,
                )


def main(args: list[str] | None = None):
    """Run the TPIK controller."""
    rclpy.init(args=args)

    node = TpikController()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
