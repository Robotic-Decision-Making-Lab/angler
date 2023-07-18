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

from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


def calculate_vehicle_angular_velocity_jacobian(rot: Rotation) -> np.ndarray:
    """Calculate the angular velocity Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.3, and is denoted there as "J_k,o".

    Args:
        rot: The rotation matrix expressing the transformation from the inertial frame
            to the body-fixed frame (i.e., map -> base_link).

    Returns:
        The vehicle's angular velocity Jacobian.
    """
    roll, pitch, _ = rot.as_euler("xyz")

    return np.array(
        [
            [1, 0, np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch) * np.sin(roll)],
            [0, -np.sin(roll), np.cos(pitch) * np.cos(roll)],
        ]
    )


def calculate_vehicle_jacobian(rot: Rotation) -> np.ndarray:
    """Calculate the Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.19, and is denoted there as "J_e".

    Args:
        rot: The rotation matrix expressing the transformation from the inertial frame
            to the body-fixed frame (i.e., map -> base_link).

    Returns:
        The vehicle Jacobian matrix.
    """
    return np.array(
        [
            [rot.as_matrix(), np.zeros((3, 3))],
            [np.zeros((3, 3)), calculate_vehicle_angular_velocity_jacobian(rot)],
        ]
    )


def calculate_manipulator_jacobian_from_dh(dh: np.ndarray) -> np.ndarray:
    """Calculate the manipulator Jacobian using its Denavit-Hartenberg parameters.

    This method is inspired by the `J_man` function defined by Gianluca Antonelli in
    the `simurv` project.

    Args:
        dh: The Denavit-Hartenberg (DH) parameters for the manipulator. This should be
            an nx4 matrix, where "n" is the total number of joints that the manipulator
            has. The parameters should be ordered as: "a", "alpha", "d", "theta". Note
            that the "theta" term should already include the current joint angles.

    Returns:
        The manipulator Jacobian.
    """

    def calculate_transform(params: np.ndarray):
        a, alpha, d, theta = params

        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Calculate the rotation matrix between consecutive links
        rot = np.array(
            [
                [ct, -st * ca, st * sa],
                [st, ct * ca, -ct * sa],
                [0, sa, ca],
            ]
        )
        p = np.array([a * np.cos(theta), a * np.sin(theta), d]).reshape((3, 1))
        transform = np.array([[rot, p], [0, 0, 0, 1]])

        return transform

    num_joints = dh.shape[0]
    J = np.zeros((6, num_joints))

    # Transform from the manipulator base frame
    T0 = np.zeros((4, 4, num_joints))

    # Positions of each joint's frame expressed in the inertial frame
    p = np.zeros((3, num_joints))

    # Z-versor of each joint's frame expressed in the inertial frame
    z = np.zeros((3, num_joints))

    # Get the base frame values
    T0[:, :, 0] = calculate_transform(dh[0])
    p[:, 0] = T0[0:2, 3, 0]
    z[:, 0] = T0[0:2, 2, 0]

    for i in range(1, num_joints):
        T_i = calculate_transform(dh[i])
        T0[:, :, i] = T0[:, :, i - 1] @ T_i
        p[:, i] = T0[0:2, 3, i]
        z[:, i] = T0[0:2, 2, i]

    z0 = np.array([0, 0, 1]).reshape((3, 1))
    p0 = np.zeros((3, 1))

    J[:, 0] = np.array([np.cross(z0, p[:, num_joints - 1] - p0)], [z0])

    for i in range(1, num_joints):
        J[:, i] = np.array(
            [np.cross(z[:, i - 1], p[:, num_joints - 1] - p[:, i - 1])], [z[:, i - 1]]
        )

    return J


def calculate_manipulator_jacobian_from_serial_chain(
    serial_chain: Any, joint_angles: list[float]
) -> np.ndarray:
    """Generate the manipulator Jacobian using a kinpy serial chain.

    Args:
        serial_chain: The kinpy serial chain.
        joint_angles: The current joint angles.

    Returns:
        The manipulator Jacobian matrix.
    """
    return np.array(serial_chain.jacobian(joint_angles))


def calculate_vehicle_roll_pitch_jacobian(
    rot: Rotation, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle roll-pitch task Jacobian.

    Args:
        rot: The rotation matrix expressing the transformation from the inertial frame
            to the body-fixed frame (i.e., map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle roll and pitch.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(rot)
    J = np.array(
        [
            np.zeros((2, 3)),
            np.array([1, 0, 0], [0, 1, 0]) @ np.linalg.pinv(J_ko),
            np.zeros((2, num_manipulator_joints)),
        ]
    )

    return J


def calculate_vehicle_yaw_jacobian(
    rot: Rotation, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle yaw task Jacobian.

    Args:
        rot: The rotation matrix expressing the transformation from the inertial frame
            to the body-fixed frame (i.e., map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle yaw.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(rot)
    J = np.array(
        [
            np.zeros((1, 3)),
            np.array([0, 0, 1]) @ np.linalg.pinv(J_ko),
            np.zeros((1, num_manipulator_joints)),
        ]
    )

    return J


def calculate_vehicle_orientation_jacobian(rot: Rotation) -> np.ndarray:
    ...
