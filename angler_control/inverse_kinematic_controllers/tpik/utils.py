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
from geometry_msgs.msg import Point, Quaternion, Vector3
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
        The manipulator Jacobian matrix.
    """
    return np.array(serial_chain.jacobian(joint_angles))
