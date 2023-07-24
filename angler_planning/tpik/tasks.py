import numpy as np
from tpik.constraint import EqualityConstraint, SetConstraint


class JointLimit(SetConstraint):
    def __init__(
        self,
        upper: float,
        lower: float,
        activation_threshold: float,
        deactivation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> None:
        super().__init__(
            "joint_limit_set",
            upper,
            lower,
            activation_threshold,
            deactivation_threshold,
            gain,
            priority,
        )
        self.joint = joint

    def calculate_jacobian(self, num_manipulator_joints: int) -> np.ndarray:
        """Calculate the Jacobian for a joint limit.

        Args:
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The joint limit Jacobian.
        """
        J = np.zeros((1, 6 + num_manipulator_joints))
        J[self.joint] = 1

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
