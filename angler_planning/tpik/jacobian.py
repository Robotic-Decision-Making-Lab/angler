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


def calculate_vehicle_roll_pitch_jacobian(
    rot_map_to_base: Quaternion, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle roll-pitch task Jacobian.

    Args:
        rot_map_to_base: The quaternion describing the rotation from the inertial frame
            to the vehicle-fixed frame (map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle roll and pitch.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

    J = np.zeros((2, 6 + num_manipulator_joints))
    J[:, 3:6] = np.array([[1, 0, 0], [0, 1, 0]]) @ np.linalg.pinv(J_ko)

    return J


def calculate_vehicle_yaw_jacobian(
    rot_map_to_base: Quaternion, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle yaw task Jacobian.

    Args:
        rot_map_to_base: The quaternion describing the rotation from the inertial frame
            to the vehicle-fixed frame (map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle yaw.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

    J = np.zeros((1, 6 + num_manipulator_joints))

    # J_ko is square, so we should be able to take the inverse of this instead of the
    # pseudoinverse
    J[:, 3:6] = np.array([0, 0, 1]) @ np.linalg.inv(J_ko)

    return J


def calculate_uvms_jacobian(
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
        transform_map_to_base: The transformation from the inertial frame to the vehicle
            base frame (map -> base_link).
        transform_manipulator_base_to_base: The transformation from the manipulator
            base frame to the vehicle base frame (alpha_base_link -> base_link).
        transform_map_to_end_effector: The transformation from the inertial frame to the
            end effector frame (map -> alpha_ee_base_link).

    Returns:
        The UVMS Jacobian.
    """
    # Get the transformation translations
    # denoted as r_{from frame}{to frame}_{with respecto x frame}
    r_MB_M = point_to_array(transform_map_to_base.translation)
    r_0B_B = point_to_array(transform_manipulator_base_to_base.translation)
    r_Mee_M = point_to_array(transform_map_to_end_effector.translation)

    # Get the transformation rotations
    # denoted as R_{from frame}_{to frame}
    R_0_B = quaternion_to_rotation(
        transform_manipulator_base_to_base.rotation
    ).as_matrix()
    R_B_M = np.linalg.inv(
        quaternion_to_rotation(transform_map_to_base.rotation).as_matrix()
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


def calculate_joint_limit_jacobian(
    joint_index: int, num_manipulator_joints: int
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


def calculate_vehicle_manipulator_collision_avoidance_jacobian(
    plane: tuple[Point, Point, Point], serial_chain: Any, joint_angles: np.ndarray
):
    """Calculate the vehicle/manipulator collision avoidance Jacobian.

    This Jacobian may be used to prevent a manipulator's end-effector from
    moving past a collision plane which represents the minimum distance from the
    vehicle.

    Args:
        plane: A tuple of three distinct points which describe the plane that the
            manipulator should not pass.
        serial_chain: The manipulator's kinpy serial chain.
        joint_angles: The manipulator's current joint angles.

    Returns:
        The collision avoidance Jacobian.
    """
    p1, p2, p3 = [point_to_array(p) for p in plane]

    # Calculate the outer normal to the plane
    plane_normal = np.cross((p2 - p1), (p3 - p1)) / np.linalg.norm(  # type: ignore
        np.cross((p2 - p1), (p3 - p1))  # type: ignore
    )

    # Get the manipulator's Jacobian
    J_pos = calculate_manipulator_jacobian(serial_chain, joint_angles)[:3, :]

    J = np.zeros((3, 6 + len(joint_angles)))
    J[:, 6:] = -plane_normal.T @ J_pos

    return J


def calculate_joint_configuration_jacobian(num_manipulator_joints: int) -> np.ndarray:
    """Calculate the Jacobian for the manipulator joint configuration Jacobian.

    Args:
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The joint configuration Jacobian.
    """
    J = np.zeros((1, 6 + num_manipulator_joints))
    J[6:] = 1

    return J
