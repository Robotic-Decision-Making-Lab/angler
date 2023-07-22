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
import tpik.conversions as conversions
from geometry_msgs.msg import Pose, Quaternion, Transform
from tpik.constraint import EqualityConstraint, SetConstraint


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


def calculate_vehicle_angular_velocity_jacobian(
    rot_map_to_base: Quaternion,
) -> np.ndarray:
    """Calculate the angular velocity Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.3, and is denoted there as "J_k,o".

    Args:
        rot_map_to_base: The quaternion describing the rotation from the inertial frame
            to the vehicle-fixed frame (map -> base_link).

    Returns:
        The vehicle's angular velocity Jacobian.
    """
    rot = conversions.quaternion_to_rotation(rot_map_to_base)
    roll, pitch, _ = rot.as_euler("xyz")

    return np.array(
        [
            [1, 0, -np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch) * np.sin(roll)],
            [0, -np.sin(roll), np.cos(pitch) * np.cos(roll)],
        ]  # type: ignore
    )


def calculate_manipulator_jacobian(
    serial_chain: Any, joint_angles: np.ndarray
) -> np.ndarray:
    """Generate the manipulator Jacobian using a kinpy serial chain.

    Args:
        serial_chain: The kinpy serial chain.
        joint_angles: The current joint angles.

    Returns:
        The manipulator Jacobian matrix.
    """
    return np.array(serial_chain.jacobian(joint_angles))


class EndEffectorPoseTask(EqualityConstraint):
    """Control the end-effector pose using an equality constraint."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new end-effector pose task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__("end_effector_pose_eq", gain, priority)

    def calculate_jacobian(
        self,
        serial_chain: Any,
        joint_angles: np.ndarray,
        transform_map_to_base: Transform,
        transform_manipulator_base_to_base: Transform,
        transform_map_to_end_effector: Transform,
    ) -> np.ndarray:
        """Calculate the UVMS Jacobian.

        Args:
            serial_chain: The manipulator kinpy serial chain.
            joint_angles: The manipulator's joint angles.
            transform_map_to_base: The transformation from the inertial frame to the
                vehicle base frame (map -> base_link).
            transform_manipulator_base_to_base: The transformation from the manipulator
                base frame to the vehicle base frame (alpha_base_link -> base_link).
            transform_map_to_end_effector: The transformation from the inertial frame
                to the end effector frame (map -> alpha_ee_base_link).

        Returns:
            The UVMS Jacobian.
        """
        # Get the transformation translations
        # denoted as r_{from frame}{to frame}_{with respecto x frame}
        r_MB_M = conversions.point_to_array(transform_map_to_base.translation)
        r_0B_B = conversions.point_to_array(
            transform_manipulator_base_to_base.translation
        )
        r_Mee_M = conversions.point_to_array(transform_map_to_end_effector.translation)

        # Get the transformation rotations
        # denoted as R_{from frame}_{to frame}
        R_0_B = conversions.quaternion_to_rotation(
            transform_manipulator_base_to_base.rotation
        ).as_matrix()
        R_B_M = np.linalg.inv(
            conversions.quaternion_to_rotation(
                transform_map_to_base.rotation
            ).as_matrix()
        )
        R_0_M = R_B_M @ R_0_B

        r_B0_M = R_B_M @ r_0B_B
        r_0ee_M = r_Mee_M - r_MB_M - r_B0_M  # type: ignore

        def get_skew_matrix(x: np.ndarray) -> np.ndarray:
            return np.array(
                [[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]],  # type: ignore
            )

        J = np.zeros((6, 6 + len(joint_angles)))
        J_man = calculate_manipulator_jacobian(serial_chain, joint_angles)

        # Position Jacobian
        J[:3, :3] = R_B_M
        J[:3, 3:6] = -(get_skew_matrix(r_B0_M) + get_skew_matrix(r_0ee_M)) @ R_B_M  # type: ignore # noqa
        J[:3, 6:] = R_0_M @ J_man[:3]

        # Orientation Jacobian
        J[3:6, 3:6] = R_B_M
        J[3:6, 6:] = R_0_M @ J_man[3:6]

        return J

    def calculate_reference(self, desired_pose: Pose, actual_pose: Pose) -> np.ndarray:
        """Calculate the reference signal for the controller.

        Args:
            desired_pose: The desired end-effector pose in the inertial (map) frame.
            actual_pose: The current end-effector pose in the inertial (map) frame.

        Returns:
            The reference signal to use to drive the system to the desired end effector
                pose.
        """
        pos_error = np.array(
            [
                desired_pose.position.x - actual_pose.position.x,
                desired_pose.position.y - actual_pose.position.y,
                desired_pose.position.z - actual_pose.position.z,
            ]
        ).reshape((3, 1))
        ori_error = calculate_quaternion_error(
            desired_pose.orientation, actual_pose.orientation
        )[:3].reshape((3, 1))

        error = np.vstack((pos_error, ori_error))

        return super()._calculate_reference(np.zeros((6, 1)), error)


class VehicleOrientationTask(EqualityConstraint):
    """Control the vehicle orientation using an equality constraint."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle orientation task.

        Args:
            gain: The vehicle orientation task gain.
            priority: The vehicle orientation task priority.
        """
        super().__init__("vehicle_orientation_eq", gain, priority)

    def calculate_jacobian(
        self, rot_base_to_map: Transform, num_manipulator_joints: int
    ) -> np.ndarray:
        """Calculate the vehicle orientation Jacobian.

        Args:
            rot_base_to_map: The quaternion describing the rotation from the
                vehicle-fixed frame to the inertial frame (base_link -> map).
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The vehicle orientation Jacobian.
        """
        J = np.zeros((3, 6 + num_manipulator_joints))
        J[:, 3:6] = conversions.quaternion_to_rotation(
            rot_base_to_map.rotation
        ).as_matrix()

        return J

    def calculate_reference(
        self, desired_orientation: Quaternion, actual_orientation: Quaternion
    ) -> np.ndarray:
        """Calculate the reference signal for the vehicle orientation task.

        Args:
            desired_orientation: The desired orientation.
            actual_orientation: The actual orientation.

        Returns:
            The reference signal for the vehicle orientation task.
        """
        # We only use the axis of rotation between the desired and current frames
        # because this vector will become null when the error is zero.
        return super()._calculate_reference(
            np.zeros((3, 1)),
            calculate_quaternion_error(desired_orientation, actual_orientation)[
                :3
            ].reshape((3, 1)),
        )


class VehicleRollPitchTask(EqualityConstraint):
    """Control the vehicle roll and pitch angles."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle roll-pitch task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__("vehicle_roll_pitch_eq", gain, priority)

    def calculate_jacobian(
        self, rot_map_to_base: Quaternion, num_manipulator_joints: int
    ) -> np.ndarray:
        """Calculate the vehicle roll-pitch task Jacobian.

        Args:
            rot_map_to_base: The quaternion describing the rotation from the inertial
                frame to the vehicle-fixed frame (map -> base_link).
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The Jacobian for a task which controls the vehicle roll and pitch.
        """
        J_ko = calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

        J = np.zeros((2, 6 + num_manipulator_joints))
        J[:, 3:6] = np.array([[1, 0, 0], [0, 1, 0]]) @ np.linalg.pinv(J_ko)

        return J

    def calculate_reference(
        self,
        desired_roll: float,
        actual_roll: float,
        desired_pitch: float,
        actual_pitch: float,
    ) -> np.ndarray:
        """Calculate the reference signal for the vehicle roll-pitch task.

        Args:
            desired_roll: The desired roll angle.
            actual_roll: The actual roll angle.
            desired_pitch: The desired pitch angle.
            actual_pitch: The actual pitch angle.

        Returns:
            The reference signal that will drive the system to the desired vehicle roll
                and pitch angles.
        """
        error = np.array(
            [desired_roll - actual_roll, desired_pitch - actual_pitch]
        ).reshape((2, 1))

        return super()._calculate_reference(np.zeros((2, 1)), error)


class VehicleYawTask(EqualityConstraint):
    """Control the vehicle yaw angle."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle yaw task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__("vehicle_roll_pitch_eq", gain, priority)

    def calculate_jacobian(
        self, rot_map_to_base: Quaternion, num_manipulator_joints: int
    ) -> np.ndarray:
        """Calculate the vehicle yaw task Jacobian.

        Args:
            rot_map_to_base: The quaternion describing the rotation from the inertial
                frame to the vehicle-fixed frame (map -> base_link).
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The Jacobian for a task which controls the vehicle yaw.
        """
        J_ko = calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

        J = np.zeros((1, 6 + num_manipulator_joints))

        # J_ko is square, so we should be able to take the inverse of this instead of
        # the pseudoinverse
        J[:, 3:6] = np.array([0, 0, 1]) @ np.linalg.inv(J_ko)

        return J

    def calculate_reference(
        self,
        desired_yaw: float,
        actual_yaw: float,
    ) -> np.ndarray:
        """Calculate the reference signal for the vehicle yaw task.

        Args:
            desired_yaw: The desired yaw angle.
            actual_yaw: The actual yaw angle.

        Returns:
            The reference signal that will drive the system to the desired vehicle yaw
                angle.
        """
        error = np.array([desired_yaw - actual_yaw])

        return super()._calculate_reference(np.zeros((1, 1)), error)


class JointLimitTask(SetConstraint):
    """Limit a joint angle to a desired range."""

    def __init__(
        self,
        upper: float,
        lower: float,
        activation_threshold: float,
        deactivation_threshold: float,
        gain: float,
        priority: float,
    ) -> None:
        """Create a new joint limit task.

        Args:
            upper: The upper joint angle limit.
            lower: The lower joint angle limit.
            activation_threshold: The threshold at which the task is activated.
            deactivation_threshold: The distance from the set boundaries at which the
                task becomes deactivated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
        """
        super().__init__(
            "joint_limit_set",
            upper,
            lower,
            activation_threshold,
            deactivation_threshold,
            gain,
            priority,
        )

    def calculate_jacobian(
        self, joint_index: int, num_manipulator_joints: int
    ) -> np.ndarray:
        """Calculate the Jacobian for a joint limit.

        Args:
            joint_index: The joint's index within the system Jacobian.
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The joint limit Jacobian.
        """
        J = np.zeros((1, 6 + num_manipulator_joints))
        J[joint_index] = 1

        return J

    def calculate_reference(
        self, desired_angle: float, actual_angle: float
    ) -> np.ndarray:
        """Calculate the reference signal for the joint limit task.

        Args:
            desired_angle: The desired joint angle.
            actual_angle: The actual joint angle.

        Returns:
            The reference signal to use to drive the joint back to a safe range.
        """
        return super()._calculate_reference(
            np.zeros((1, 1)), np.array([desired_angle - actual_angle])
        )


class ManipulatorConfigurationTask(EqualityConstraint):
    """Set the manipulator joint angles."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new joint configuration task.

        Args:
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
        """
        super().__init__("joint_configuration_eq", gain, priority)

    def calculate_jacobian(self, num_manipulator_joints: int) -> np.ndarray:
        """Calculate the Jacobian for the manipulator joint configuration Jacobian.

        Args:
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The joint configuration Jacobian.
        """
        J = np.zeros((1, 6 + num_manipulator_joints))
        J[6:] = 1

        return J

    def calculate_reference(
        self, desired_configuration: np.ndarray, actual_configuration: np.ndarray
    ) -> np.ndarray:
        """Calculate the reference signal for the joint configuration task.

        Args:
            desired_angle: The desired joint angles.
            actual_angle: The actual joint angles.

        Returns:
            The reference signal to use to set the manipulator joint angles.
        """
        return super()._calculate_reference(
            np.zeros(desired_configuration.shape),
            desired_configuration - actual_configuration,
        )
