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
from geometry_msgs.msg import Point, Quaternion, Transform, Vector3
from scipy.spatial.transform import Rotation as R


def quaternion_to_rotation(quat: Quaternion) -> R:
    """Convert a Quaternion message into a SciPy Rotation.

    Args:
        quat: The Quaternion message to convert.

    Returns:
        The resulting SciPy Rotation.
    """
    return R.from_quat([quat.x, quat.y, quat.z, quat.w])


def point_to_array(point: Point | Vector3) -> np.ndarray:
    """Convert a Point message into a numpy array.

    Args:
        point: The Point message to convert.

    Returns:
        The resulting point as a 3x1 numpy array.
    """
    return np.array([point.x, point.y, point.z]).reshape((3, 1))


def get_skew_matrix(x: np.ndarray) -> np.ndarray:
    """Convert a 3x1 matrix of coefficients into a skew-symmetric matrix.

    Args:
        x: The matrix to convert.

    Returns:
        A skew-symmetric matrix.
    """
    return np.array(
        [
            [0, -x[2][0], x[1][0]],  # type: ignore
            [x[2][0], 0, -x[0][0]],
            [-x[1][0], x[0][0], 0],
        ],
    )


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
    rot = quaternion_to_rotation(rot_map_to_base)
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
        The 6xn manipulator Jacobian matrix.
    """
    return np.array(serial_chain.jacobian(joint_angles))


def calculate_vehicle_roll_pitch_jacobian(rot_map_to_base: Quaternion) -> np.ndarray:
    """Calculate the vehicle roll-pitch Jacobian.

    Args:
        rot_map_to_base: A quaternion which defines the rotation from the map frame to
            the base frame.

    Returns:
        The 2x6 vehicle roll-pitch Jacobian.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

    J = np.zeros((2, 6))
    J[:, 3:6] = np.array([[1, 0, 0], [0, 1, 0]]) @ np.linalg.pinv(J_ko)

    return J


def calculate_vehicle_yaw_jacobian(rot_map_to_base: Quaternion) -> np.ndarray:
    """Calculate the vehicle yaw Jacobian.

    Args:
        rot_map_to_base: A quaternion which defines the rotation from the map frame to
            the base frame.

    Returns:
        The 1x6 vehicle yaw Jacobian.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

    J = np.zeros((1, 6))
    J[:, 3:6] = np.array([0, 0, 1]) @ np.linalg.inv(J_ko)

    return J


def calculate_vehicle_orientation_jacobian(rot_base_to_map: Quaternion) -> np.ndarray:
    """Calculate the vehicle orientation Jacobian.

    Args:
        rot_base_to_map: A quaternion which defines the rotation from the base frame to
            the map frame.

    Returns:
        The 3x6 vehicle orientation Jacobian
    """
    J = np.zeros((3, 6))
    J[:, 3:6] = quaternion_to_rotation(rot_base_to_map).as_matrix()

    return J


def calculate_uvms_jacobian(
    tf_base_to_manipulator_base: Transform,
    tf_manipulator_base_to_ee: Transform,
    tf_map_to_base: Transform,
    n_manipulator_joints: int,
    serial_chain: Any,
    joint_angles: np.ndarray,
) -> np.ndarray:
    """Calculate the full UVMS Jacobian.

    Args:
        tf_base_to_manipulator_base: A Transform describing the transformation from the
            base frame to the manipulator base frame with respect to the base frame.
        tf_manipulator_base_to_ee: A Transform describing the transformation from the
            manipulator base frame to the end-effector frame with respect to the
            manipulator base frame.
        tf_map_to_base: A Transform describing the transformation from the
            map frame to the vehicle base frame with respect to the map frame.
        n_manipulator_joints: The number of joints that the manipulator has.
        serial_chain: The kinpy serial chain.
        joint_angles: The current manipulator joint angles.

    Returns:
        The 3x6+n UVMS Jacobian.
    """
    # Get the transformation translations
    # denoted as r_{from frame}{to frame}_{with respect to x frame}
    r_B0_B = point_to_array(tf_base_to_manipulator_base.translation)
    eta_0ee_0 = point_to_array(tf_manipulator_base_to_ee.translation)

    # Get the transformation rotations
    # denoted as R_{from frame}_{to frame}
    R_0_B = np.linalg.inv(
        quaternion_to_rotation(tf_base_to_manipulator_base.rotation).as_matrix()
    )
    R_B_I = np.linalg.inv(quaternion_to_rotation(tf_map_to_base.rotation).as_matrix())
    R_0_I = R_B_I @ R_0_B

    r_B0_I = R_B_I @ r_B0_B
    eta_0ee_I = R_0_I @ eta_0ee_0

    J = np.zeros((6, 6 + n_manipulator_joints))
    J_man = calculate_manipulator_jacobian(serial_chain, joint_angles)

    # Position Jacobian
    J[:3, :3] = R_B_I
    J[:3, 3:6] = -(get_skew_matrix(r_B0_I) + get_skew_matrix(eta_0ee_I)) @ R_B_I  # type: ignore # noqa
    J[:3, 6:] = R_0_I @ J_man[:3]

    # Orientation Jacobian
    J[3:6, 3:6] = R_B_I
    J[3:6, 6:] = R_0_I @ J_man[3:]

    return J


def calculate_joint_configuration_jacobian(n_manipulator_joints: int) -> np.ndarray:
    """Calculate the joint configuration Jacobian.

    Args:
        n_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The nxn manipulator configuration Jacobian.
    """
    return np.eye(n_manipulator_joints)
