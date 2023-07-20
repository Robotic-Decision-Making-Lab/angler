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
from geometry_msgs.msg import Point, Transform

import angler_planning.tpik.conversions as conversions


def calculate_vehicle_angular_velocity_jacobian(
    transform_map_to_base: Transform,
) -> np.ndarray:
    """Calculate the angular velocity Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.3, and is denoted there as "J_k,o".

    Args:
        transform_map_to_base: The transformation from the inertial frame to the
            body-fixed frame (map -> base_link).

    Returns:
        The vehicle's angular velocity Jacobian.
    """
    rot = conversions.quaternion_to_rotation(transform_map_to_base.rotation)
    roll, pitch, _ = rot.as_euler("xyz")

    return np.array(
        [
            [1, 0, np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch) * np.sin(roll)],
            [0, -np.sin(roll), np.cos(pitch) * np.cos(roll)],
        ]
    )


def calculate_vehicle_jacobian(transform_map_to_base: Transform) -> np.ndarray:
    """Calculate the Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.19, and is denoted there as "J_e".

    Args:
        transform_map_to_base: The transformation from the inertial frame to the
            body-fixed frame (map -> base_link).

    Returns:
        The vehicle Jacobian matrix.
    """
    rot = conversions.quaternion_to_rotation(transform_map_to_base.rotation)

    J = np.zeros((6, 6))

    J[0:3, 0:3] = rot.as_matrix()
    J[3:6, 3:6] = calculate_vehicle_angular_velocity_jacobian(transform_map_to_base)

    return J


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
    r_MB_M = conversions.point_to_array(transform_map_to_base.translation)
    r_0B_B = conversions.point_to_array(transform_manipulator_base_to_base.translation)
    r_Mee_M = conversions.point_to_array(transform_map_to_end_effector.translation)

    # Get the transformation rotations
    # denoted as R_{from frame}_{to frame}
    R_0_B = conversions.quaternion_to_rotation(
        transform_manipulator_base_to_base.rotation
    ).as_matrix()
    R_B_M = np.linalg.inv(
        conversions.quaternion_to_rotation(transform_map_to_base.rotation).as_matrix()
    )
    R_0_M = R_B_M @ R_0_B

    r_B0_M = R_B_M @ r_0B_B
    r_0ee_I = r_Mee_M - r_MB_M - r_B0_M  # type: ignore

    def get_skew_matrix(x: np.ndarray) -> np.ndarray:
        return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

    J = np.zeros((6, 6 + len(joint_angles)))
    J_man = calculate_manipulator_jacobian(serial_chain, joint_angles)

    # Position Jacobian
    J[0:3, 0:3] = R_B_M
    J[0:3, 3:6] = -(get_skew_matrix(r_B0_M) + get_skew_matrix(r_0ee_I)) @ R_B_M  # type: ignore # noqa
    J[0:3, 6:] = R_0_M @ J_man[0:3]

    # Orientation Jacobian
    J[3:6, 3:6] = R_B_M
    J[3:6, 6:] = R_0_M @ J_man[3:6]

    return J


def calculate_vehicle_roll_pitch_jacobian(
    transform_map_to_base: Transform, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle roll-pitch task Jacobian.

    Args:
        transform_map_to_base: The transformation from the inertial frame to the
            body-fixed frame (map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle roll and pitch.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(transform_map_to_base)

    J = np.array(
        [
            np.zeros((2, 3)),
            np.array([1, 0, 0], [0, 1, 0]) @ np.linalg.pinv(J_ko),
            np.zeros((2, num_manipulator_joints)),
        ]
    )

    return J


def calculate_vehicle_yaw_jacobian(
    transform_map_to_base: Transform, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle yaw task Jacobian.

    Args:
        transform_map_to_base: The transformation from the inertial frame to the
            body-fixed frame (map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle yaw.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(transform_map_to_base)
    J = np.array(
        [
            np.zeros((1, 3)),
            np.array([0, 0, 1]) @ np.linalg.pinv(J_ko),
            np.zeros((1, num_manipulator_joints)),
        ]
    )

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
    p1: Point, p2: Point, p3: Point, serial_chain: Any, joint_angles: np.ndarray
):
    """Calculate the vehicle/manipulator collision avoidance Jacobian.

    This Jacobian may be used to prevent a manipulator's end-effector from
    moving past a collision plane which represents the minimum distance from the
    vehicle.

    Args:
        p1: The first point on the collision plane.
        p2: The second point on the collision plane.
        p3: The third point on the collision plane.
        serial_chain: The manipulator's kinpy serial chain.
        joint_angles: The manipulator's current joint angles.

    Returns:
        The collision avoidance Jacobian.
    """
    # Convert the points to numpy arrays
    p1_ar = conversions.point_to_array(p1)
    p2_ar = conversions.point_to_array(p2)
    p3_ar = conversions.point_to_array(p3)

    # Calculate the outer normal to the plane
    plane_normal = np.cross((p2_ar - p1_ar), (p3_ar - p1_ar)) / np.linalg.norm(  # type: ignore # noqa
        np.cross((p2_ar - p1_ar), (p3_ar - p1_ar))  # type: ignore
    )

    # Get the manipulator's vehicle Jacobian
    J_pos = calculate_manipulator_jacobian(serial_chain, joint_angles)[:3, :]

    J = np.zeros((3, 6 + len(joint_angles)))
    J[:, 6:] = -plane_normal.T @ J_pos

    return J
