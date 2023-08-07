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

from typing import Any

import numpy as np
from geometry_msgs.msg import Quaternion, Transform
from inverse_kinematic_controllers.tpik.constraint import (
    EqualityConstraint,
    SetConstraint,
)
from scipy.spatial.transform import Rotation as R

import angler_kinematics.jacobian as jacobian  # type: ignore


def calculate_quaternion_error(desired: Quaternion, actual: Quaternion) -> np.ndarray:
    """Calculate the error between two quaternions.

    Args:
        desired: The desired orientation.
        actual: The actual orientation.

    Returns:
        The angle between the two quaternions as a quaternion.
    """

    def quat_to_array(quat: Quaternion):
        return np.array([quat.x, quat.y, quat.z, quat.w])

    quat_d = quat_to_array(desired)
    quat_a = quat_to_array(actual)

    error_eps = (
        quat_a[3] * quat_d[:3]
        - quat_d[3] * quat_a[:3]
        + np.cross(quat_a[:3], quat_d[:3])
    )
    error_lambda = quat_d[3] * quat_a[3] + quat_d[:3].T @ quat_a[:3]

    return np.hstack((error_eps, error_lambda))


class VehicleRollPitchTask(EqualityConstraint):
    """Control the vehicle roll and pitch angles."""

    name = "vehicle_roll_pitch_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle roll pitch task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.desired_value = np.zeros((2, 1))
        self.current_value = np.zeros((2, 1))
        self.n_manipulator_joints = 0
        self.rot_map_to_base = Quaternion()

    @staticmethod
    def create_task_from_params(
        gain: float,
        priority: float,
        roll: float | None = None,
        pitch: float | None = None,
    ) -> Any:
        """Create a vehicle roll-pitch task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            roll: The desired roll angle, if known beforehand. Defaults to None.
            pitch: The desired pitch angle, if known beforehand. Defaults to None.

        Returns:
            A vehicle roll-pitch task.
        """
        task = VehicleRollPitchTask(gain, priority)

        if None not in [roll, pitch]:
            task.desired_value = np.array([roll, pitch]).reshape((2, 1))

        return task

    def update(
        self,
        current_rot: Quaternion,
        desired_rot: Quaternion | None = None,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the task context.

        Args:
            current_rot: The current vehicle orientation in the inertial (map)
                frame.
            desired_rot: The desired vehicle orientation. The desired roll and
                pitch angles will be extracted from the rotation. Defaults to None.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.rot_map_to_base = current_rot

        # Remove the yaw from the quaternions and get the roll, pitch angles
        cr = R.from_quat(
            [
                current_rot.x,
                current_rot.y,
                current_rot.z,
                current_rot.w,
            ]
        )
        current_roll, current_pitch, _ = cr.as_euler("xyz")
        self.current_value = np.array([current_roll, current_pitch]).reshape((2, 1))

        if desired_rot is not None:
            dr = R.from_quat(
                [
                    desired_rot.x,
                    desired_rot.y,
                    desired_rot.z,
                    desired_rot.w,
                ]
            )
            desired_roll, desired_pitch, _ = dr.as_euler("xyz")
            self.desired_rotation_rotation = np.array(
                [desired_roll, desired_pitch]
            ).reshape((2, 1))

        # The number of manipulator joints doesn't need to change every update
        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the vehicle roll-pitch task Jacobian.

        Returns:
            The Jacobian for a task which controls the vehicle roll and pitch.
        """
        J = np.zeros((2, 6 + self.n_manipulator_joints))
        J[:, :6] = jacobian.calculate_vehicle_roll_pitch_jacobian(self.rot_map_to_base)

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle roll-pitch task.

        Returns:
            The reference signal.
        """
        return self.desired_value - self.current_value  # type: ignore


class VehicleYawTask(EqualityConstraint):
    """Control the vehicle yaw angle."""

    name = "vehicle_yaw_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle yaw task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.current_value = np.zeros((1, 1))
        self.desired_value = np.zeros((1, 1))
        self.rot_map_to_base = Quaternion()
        self.n_manipulator_joints = 0

    @staticmethod
    def create_task_from_params(
        gain: float, priority: float, yaw: float | None = None
    ) -> Any:
        """Create a new vehicle yaw task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            yaw: The desired vehicle yaw, if known beforehand. Defaults to None.
        """
        task = VehicleYawTask(gain, priority)

        if yaw is not None:
            task.desired_value = np.array([yaw])

        return task

    def update(
        self,
        current_rot: Quaternion,
        desired_rot: Quaternion | None = None,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the context for the yaw task.

        Args:
            current_rot: The current vehicle orientation in the inertial (map)
                frame.
            desired_rot: The desired vehicle orientation--the desired yaw angle
                will be extracted from the quaternion. Defaults to None.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.rot_map_to_base = current_rot

        # Remove the roll, pitch from the quaternion and get the yaw angle
        cr = R.from_quat(
            [
                current_rot.x,
                current_rot.y,
                current_rot.z,
                current_rot.w,
            ]
        )
        _, _, current_yaw = cr.as_euler("xyz")
        self.current_value = np.array([current_yaw])

        if desired_rot is not None:
            dr = R.from_quat(
                [
                    desired_rot.x,
                    desired_rot.y,
                    desired_rot.z,
                    desired_rot.w,
                ]
            )
            _, _, desired_yaw = dr.as_euler("xyz")
            self.desired_value = np.array([desired_yaw])

        # The total number of manipulator joints doesn't need to be updated at each
        # iteration
        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the vehicle yaw task Jacobian.

        Returns:
            The Jacobian for a task which controls the vehicle yaw.
        """
        J = np.zeros((1, 6 + self.n_manipulator_joints))
        J[:, :6] = jacobian.calculate_vehicle_yaw_jacobian(self.rot_map_to_base)

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle yaw task.

        Returns:
            The reference signal.
        """
        return self.desired_value - self.current_value  # type: ignore


class EndEffectorPoseTask(EqualityConstraint):
    """Control the end-effector pose."""

    name = "end_effector_pose_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new end effector pose task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.serial_chain: Any | None = None
        self.joint_angles = np.zeros((1, 1))
        self.tf_map_to_base = Transform()
        self.tf_base_to_manipulator_base = Transform()
        self.tf_manipulator_base_to_ee = Transform()
        self.current_value = Transform()
        self.desired_value = Transform()

    @staticmethod
    def create_task_from_params(
        gain: float,
        priority: float,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> Any:
        """Create a new end effector pose task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            x: The desired end effector x position in the inertial (map) frame. Defaults
                to None.
            y: The desired end effector y position in the inertial (map) frame. Defaults
                to None.
            z: The desired end effector z position in the inertial (map) frame. Defaults
                to None.
            roll: The desired end effector roll in the inertial (map) frame. Defaults
                to None.
            pitch: The desired end effector pitch in the inertial (map) frame. Defaults
                to None.
            yaw: The desired end effector yaw in the inertial (map) frame. Defaults
                to None.

        Returns:
            A new end effector pose task.
        """
        task = EndEffectorPoseTask(gain, priority)

        if None not in [x, y, z, roll, pitch, yaw]:
            task.desired_value.translation.x = x  # type: ignore
            task.desired_value.translation.y = y  # type: ignore
            task.desired_value.translation.z = z  # type: ignore

            (
                task.desired_value.rotation.x,  # type: ignore
                task.desired_value.rotation.y,  # type: ignore
                task.desired_value.rotation.z,  # type: ignore
                task.desired_value.rotation.w,  # type: ignore
            ) = R.from_euler(
                "xyz", [roll, pitch, yaw]  # type: ignore
            ).as_quat()

        return task

    def update(
        self,
        joint_angles: np.ndarray,
        tf_map_to_ee: Transform,
        tf_map_to_base: Transform,
        tf_base_to_manipulator_base: Transform,
        tf_manipulator_base_to_ee: Transform,
        serial_chain: Any | None = None,
        desired_pose: Transform | None = None,
    ) -> None:
        """Update the current context of the end effector pose task.

        Args:
            joint_angles: The current manipulator joint angles.
            current_pose: The current vehicle pose in the inertial (map) frame.
            tf_map_to_ee: The transformation from the inertial (map) frame to the
                end effector frame.
            tf_map_to_base: The transformation from the inertial (map) frame to the
                vehicle base frame.
            tf_base_to_manipulator_base: The transformation from the vehicle base frame
                to the manipulator base frame.
            tf_manipulator_base_to_ee: The transformation from the manipulator base
                frame to the end effector frame.
            serial_chain: The manipulator kinpy serial chain. Defaults to None.
            desired_pose: The desired end effector pose. Defaults to None.
        """
        self.joint_angles = joint_angles
        self.current_value = tf_map_to_ee
        self.tf_map_to_base = tf_map_to_base
        self.tf_base_to_manipulator_base = tf_base_to_manipulator_base
        self.tf_manipulator_base_to_ee = tf_manipulator_base_to_ee

        if serial_chain is not None:
            self.serial_chain = serial_chain

        if desired_pose is not None:
            self.desired_value = desired_pose

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the UVMS Jacobian.

        Returns:
            The UVMS Jacobian.
        """
        return jacobian.calculate_uvms_jacobian(
            self.tf_base_to_manipulator_base,
            self.tf_manipulator_base_to_ee,
            self.tf_map_to_base,
            len(self.joint_angles),
            self.serial_chain,
            self.joint_angles,
        )

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the controller.

        Returns:
            The reference signal to use to drive the system to the desired end effector
                pose.
        """
        pos_error = np.array(
            [
                self.desired_value.translation.x - self.current_value.translation.x,  # type: ignore # noqa
                self.desired_value.translation.y - self.current_value.translation.y,  # type: ignore # noqa
                self.desired_value.translation.z - self.current_value.translation.z,  # type: ignore # noqa
            ]
        ).reshape((3, 1))
        ori_error = calculate_quaternion_error(
            self.desired_value.rotation, self.current_value.rotation  # type: ignore
        )[:3].reshape((3, 1))

        return np.vstack((pos_error, ori_error))


class ManipulatorJointLimitTask(SetConstraint):
    """Limit a manipulator joint angle to a desired range."""

    name = "joint_limit_set"

    def __init__(
        self,
        physical_upper: float,
        physical_lower: float,
        safety_upper: float,
        safety_lower: float,
        activation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> None:
        """Create a new joint limit set constraint.

        Args:
            physical_upper: The maximum joint angle.
            physical_lower: The minimum joint angle.
            safety_upper: The upper safety limit used to create a buffer from the
                upper physical limit.
            safety_lower: The lower safety limit used to create a buffer from the lower
                physical limit.
            activation_threshold: The distance from safety thresholds at which the task
                should become activated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
            joint: The joint that the constraint applies to.
        """
        super().__init__(
            physical_upper,
            physical_lower,
            safety_upper,
            safety_lower,
            activation_threshold,
            gain,
            priority,
        )

        self.joint = joint
        self.n_manipulator_joints = 0
        self.current_value = 0.0
        self.desired_value = 0.0

    @staticmethod
    def create_task_from_params(
        physical_upper: float,
        physical_lower: float,
        safety_upper: float,
        safety_lower: float,
        activation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> Any:
        """Create a new joint limit task.

        Args:
            physical_upper: The maximum joint angle.
            physical_lower: The minimum joint angle.
            safety_upper: The upper safety limit used to create a buffer from the
                upper physical limit.
            safety_lower: The lower safety limit used to create a buffer from the lower
                physical limit.
            activation_threshold: The distance from safety thresholds at which the task
                should become activated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
            joint: The joint that the constraint applies to.
        """
        return ManipulatorJointLimitTask(
            physical_upper,
            physical_lower,
            safety_upper,
            safety_lower,
            activation_threshold,
            gain,
            priority,
            joint,
        )

    def update(
        self, current_angle: float, n_manipulator_joints: int | None = None
    ) -> None:
        """Update the current context of the joint limit task.

        Args:
            current_angle: The current joint angle.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.current_value = current_angle

        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the Jacobian for a joint limit.

        Args:
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The joint limit Jacobian.
        """
        J = np.zeros((1, 6 + self.n_manipulator_joints))
        J[0, self.joint] = 1

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the joint limit task.

        Returns:
            The reference signal to use to drive the joint back to a safe range.
        """
        return np.array([self.desired_value - self.current_value])  # type: ignore


class ManipulatorJointConfigurationTask(EqualityConstraint):
    """Control the joint angles of the manipulator."""

    name = "manipulator_configuration_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new manipulator configuration task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.current_value = np.zeros((1, 1))
        self.desired_value = np.zeros((1, 1))
        self.n_manipulator_joints = 0

    @staticmethod
    def create_task_from_params(
        gain: float, priority: float, desired_joint_angles: list[float] | None
    ) -> Any:
        """Create a new manipulator configuration task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            desired_joint_angles: The desired joint angles. Defaults to None.

        Returns:
            A new manipulator configuration task.
        """
        task = ManipulatorJointConfigurationTask(gain, priority)

        if desired_joint_angles is not None:
            task.desired_value = np.array(desired_joint_angles).reshape(
                (len(desired_joint_angles), 1)
            )

        return task

    def update(
        self,
        current_joint_angles: np.ndarray,
        desired_joint_angles: np.ndarray | None = None,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the current context for the manipulator configuration task.

        Args:
            current_joint_angles: The current joint angles.
            desired_joint_angles: The desired joint angles. Defaults to None.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.current_value = current_joint_angles

        if desired_joint_angles is not None:
            self.desired_value = desired_joint_angles

        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the Jacobian for the manipulator joint configuration task.

        Returns:
            The joint configuration Jacobian.
        """
        J = np.zeros((self.n_manipulator_joints, 6 + self.n_manipulator_joints))
        J[0:, 6:] = jacobian.calculate_joint_configuration_jacobian(
            self.n_manipulator_joints
        )

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference for the joint configuration task.

        Returns:
            The joint configuration reference signal.
        """
        return self.desired_value - self.current_value  # type: ignore


# Maintain a dictionary of tasks so that it's easier to generate them in the hierarchy
task_library = {
    task.name: task  # type: ignore
    for task in [
        EndEffectorPoseTask,
        ManipulatorJointLimitTask,
        ManipulatorJointConfigurationTask,
        VehicleRollPitchTask,
        VehicleYawTask,
    ]
}
