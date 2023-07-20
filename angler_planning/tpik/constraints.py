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

from abc import ABC, abstractmethod
from typing import Any, Callable

import numpy as np
from geometry_msgs.msg import Pose, Transform, Twist

import angler_planning.tpik.jacobian as J


class Constraint(ABC):
    """Base class for defining a constraint."""

    def __init__(
        self, gain: float, priority: float, jacobian_func: Callable[[], np.ndarray]
    ) -> None:
        """Create a new constraint.

        Args:
            gain: The constraint gain.
            priority: The constraint priority in the constraint priority list.
            jacobian_func: The function used to calculate the constraint Jacobian.
        """
        self.gain = gain
        self.priority = priority
        self._jacobian_func = jacobian_func

    @abstractmethod
    def calculate_jacobian(self, *args, **kwargs) -> np.ndarray:
        """Calculate the constraint Jacobian.

        Returns:
            The constraint Jacobian.
        """
        return self._jacobian_func(*args, **kwargs)

    @abstractmethod
    def calculate_reference(
        self, feedforward: np.ndarray, error: np.ndarray
    ) -> np.ndarray:
        """Calculate the reference signal for a task.

        Args:
            feedforward: The reference signal feedforward term. This is typically the
                dynamics of the task value.
            error: The current task error.

        Returns:
            The task reference signal.
        """
        return feedforward + self.gain @ error

    @staticmethod
    def calculate_nullspace(augmented_jacobian: np.ndarray) -> np.ndarray:
        """Calculate the nullspace of the augmented Jacobian.

        Args:
            augmented_jacobian: The augmented Jacobian whose nullspace will be projected
                into.

        Returns:
            The nullspace of the augmented Jacobian.
        """
        return (
            np.eye(augmented_jacobian.shape[0], augmented_jacobian.shape[1])
            - np.linalg.pinv(augmented_jacobian) @ augmented_jacobian
        )


class EqualityConstraint(Constraint):
    def __init__(
        self, gain: float, priority: float, jacobian_func: Callable[[], np.ndarray]
    ) -> None:
        super().__init__(gain, priority, jacobian_func)


class SetConstraint(Constraint):
    def __init__(
        self,
        min_value: float,
        max_value: float,
        gain: float,
        priority: float,
        jacobian_func: Callable[[], np.ndarray],
    ) -> None:
        super().__init__(gain, priority, jacobian_func)


class EndEffectorPoseConstraint(EqualityConstraint):
    def __init__(self, gain: float, priority: float) -> None:
        super().__init__(gain, priority, J.calculate_uvms_jacobian)  # type: ignore

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
        return super().calculate_jacobian(
            serial_chain,
            joint_angles,
            transform_map_to_base,
            transform_manipulator_base_to_base,
            transform_map_to_end_effector,
        )

    def calculate_reference(
        self, feedforward: Twist, desired_pose: Pose, actual_pose: Pose
    ) -> np.ndarray:
        feedforward_ar = np.array(
            [
                feedforward.linear.x,
                feedforward.linear.y,
                feedforward.linear.z,
                feedforward.angular.z,
                feedforward.angular.y,
                feedforward.angular.z,
            ]
        ).reshape((6, 1))

        pos_error = np.array(
            [
                desired_pose.position.x - actual_pose.position.x,
                desired_pose.position.y - actual_pose.position.y,
                desired_pose.position.z - actual_pose.position.z,
            ]
        ).reshape((3, 1))

        quat_d = np.array(
            [
                desired_pose.orientation.x,
                desired_pose.orientation.y,
                desired_pose.orientation.z,
                desired_pose.orientation.w,
            ]
        )

        quat_a = np.array(
            [
                actual_pose.orientation.x,
                actual_pose.orientation.y,
                actual_pose.orientation.z,
                actual_pose.orientation.w,
            ]
        )

        ori_error = (
            quat_a[3] * quat_d[0:3]
            - quat_d[3] * quat_a[0:3]
            + np.cross(quat_a[0:3], quat_d[0:3])
        )

        error = np.zeros((6, 1))
        error[0:3] = pos_error
        error[3:6] = ori_error

        return super().calculate_reference(feedforward_ar, error)
