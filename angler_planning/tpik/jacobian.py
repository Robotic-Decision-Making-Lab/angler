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
from geometry_msgs.msg import Point, Pose, Transform
from scipy.spatial.transform.rotation import Rotation as R

import angler_planning.tpik.conversions as conversions


def calculate_transform(joint_dh_params: np.ndarray) -> np.ndarray:
    """Construct a homogeneous transformation matrix from using a joint's DH params."""
    a, alpha, d, theta = joint_dh_params

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

    # Construct the homogenous transformation matrix
    transform = np.zeros((4, 4))
    transform[0:3, 0:3] = rot
    transform[0:3, 3] = p
    transform[3, 3] = 1

    return transform


def calculate_vehicle_angular_velocity_jacobian(transform: Transform) -> np.ndarray:
    """Calculate the angular velocity Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.3, and is denoted there as "J_k,o".

    Args:
        transform: The transformation from the inertial frame to the body-fixed frame
            (i.e., map -> base_link).

    Returns:
        The vehicle's angular velocity Jacobian.
    """
    rot = conversions.quaternion_to_rotation(transform.rotation)
    roll, pitch, _ = rot.as_euler("xyz")

    return np.array(
        [
            [1, 0, np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch) * np.sin(roll)],
            [0, -np.sin(roll), np.cos(pitch) * np.cos(roll)],
        ]
    )


def calculate_vehicle_jacobian(transform: Transform) -> np.ndarray:
    """Calculate the Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.19, and is denoted there as "J_e".

    Args:
        transform: The transformation from the inertial frame to the body-fixed frame
            (i.e., map -> base_link).

    Returns:
        The vehicle Jacobian matrix.
    """
    rot = conversions.quaternion_to_rotation(transform.rotation)

    J = np.zeros((6, 6))

    J[0:3, 0:3] = rot.as_matrix()
    J[3:6, 3:6] = calculate_vehicle_angular_velocity_jacobian(transform)

    return J


def calculate_manipulator_jacobian(
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


def calculate_uvms_jacobian(
    vehicle_pose: Pose,
    dh: np.ndarray,
    T_0_B: Transform,
) -> np.ndarray:
    """Calculate the UVMS Jacobian.

    Args:
        vehicle_pose: The current pose of the vehicle: (x, y, z, roll, pitch, yaw).
        dh: The manipulator's Denavit-Hartenburg parameters.
        T_0_B: The homogenous transformation matrix from the vehicle base frame to the
            manipulator base frame (base_link -> alpha_base_link).

    Returns:
        The resulting UVMS Jacobian.
    """
    # Perform initial conversions
    eta = conversions.pose_to_array(vehicle_pose)
    transform_ar = conversions.transform_to_array(T_0_B)

    # Split up the vehicle state
    vehicle_position = eta[0:3]
    vehicle_rpy = eta[3:6]

    # Split up the transformation matrix
    r_B0_B = transform_ar[0:3, 3]
    R_0_B = transform_ar[0:3, 0:3]

    J = np.zeros((6, 6 + dh.shape[0]))

    J_man = calculate_manipulator_jacobian_from_dh(dh)
    R_B_I = R.from_euler("xyz", vehicle_rpy.reshape((1, 3))).as_matrix()
    R_0_I = R_B_I @ R_0_B


def calculate_vehicle_roll_pitch_jacobian(
    transform: Transform, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle roll-pitch task Jacobian.

    Args:
        transform: The transformation from the inertial frame to the body-fixed frame
            (i.e., map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle roll and pitch.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(transform)

    J = np.array(
        [
            np.zeros((2, 3)),
            np.array([1, 0, 0], [0, 1, 0]) @ np.linalg.pinv(J_ko),
            np.zeros((2, num_manipulator_joints)),
        ]
    )

    return J


def calculate_vehicle_yaw_jacobian(
    transform: Transform, num_manipulator_joints: int
) -> np.ndarray:
    """Calculate the vehicle yaw task Jacobian.

    Args:
        transform: The transformation from the inertial frame to the body-fixed frame
            (i.e., map -> base_link).
        num_manipulator_joints: The total number of joints that the manipulator has.

    Returns:
        The Jacobian for a task which controls the vehicle yaw.
    """
    J_ko = calculate_vehicle_angular_velocity_jacobian(transform)
    J = np.array(
        [
            np.zeros((1, 3)),
            np.array([0, 0, 1]) @ np.linalg.pinv(J_ko),
            np.zeros((1, num_manipulator_joints)),
        ]
    )

    return J


def calculate_joint_limit_jacobian(
    joint_index: int, num_system_joints: int
) -> np.ndarray:
    """Calculate the Jacobian for a joint limit.

    Args:
        joint_index: The joint's index within the system Jacobian.
        num_system_joints: The total number of joints that the system has (e.g., 6 + n,
            where "n" is the total number of manipulator joints).

    Returns:
        The joint limit Jacobian.
    """
    J = np.zeros((1, num_system_joints))
    J[joint_index] = 1

    return J


def calculate_maximum_depth_jacobian(transform: Transform) -> np.ndarray:
    """Calculate the Jacobian for the maximum depth.

    Args:
        transform: The transformation from the inertial frame to the body-fixed frame
            (i.e., map -> base_link).

    Returns:
        The max depth Jacobian.
    """
    J = np.zeros((1, 6))

    # Set the Z-axis element to 1
    J[2] = 1

    # Multiply by the vehicle Jacobian
    J = J @ calculate_vehicle_jacobian(transform)

    return J


def calculate_vehicle_manipulator_collision_avoidance_jacobian(
    p1: Point, p2: Point, p3: Point, dh: np.ndarray
):
    """Calculate the vehicle/manipulator collision avoidance Jacobian.

    This Jacobian may be used to prevent a manipulator's end-effector from
    moving past a collision plane which represents the minimum distance from the
    vehicle.

    Args:
        p1: The first point on the collision plane.
        p2: The second point on the collision plane.
        p3: The third point on the collision plane.
        dh: The manipulator's Denavit-Hartenburg parameters.

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
    J_pos = calculate_manipulator_jacobian_from_dh(dh)[:3, :]

    J = -plane_normal.T @ J_pos

    return J
