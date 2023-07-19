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

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from scipy.spatial.transform import Rotation as R


def quaternion_to_rotation(quat: Quaternion) -> R:
    """Convert a Quaternion message into a SciPy Rotation.

    Args:
        quat: The Quaternion message to convert.

    Returns:
        The resulting SciPy Rotation.
    """
    return R.from_quat([quat.x, quat.y, quat.z, quat.w])


def point_to_array(point: Point) -> np.ndarray:
    """Convert a Point message into a numpy array.

    Args:
        point: The Point message to convert.

    Returns:
        The resulting point as a 3x1 numpy array.
    """
    return np.array([point.x, point.y, point.z]).reshape((3, 1))


def transform_to_array(transform: Transform) -> np.ndarray:
    """Convert a Transform message into a homogenous transformation matrix.

    Args:
        transform: The Transform message to convert.

    Returns:
        The resulting transformation matrix as a 4x4 numpy array.
    """
    rot = quaternion_to_rotation(transform.rotation)
    rot_mat = rot.as_matrix()

    transformation_mat = np.zeros((4, 4))
    transformation_mat[0:3, 0:3] = rot_mat
    rot_mat[0:3, 3] = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z]
    ).reshape((3, 1))
    transformation_mat[3, 3] = 1

    return transformation_mat


def pose_to_array(pose: Pose) -> np.ndarray:
    """Convert a Pose message into a numpy array compatible with the Jacobian interface.

    Args:
        pose: The Pose message to convert into an array.

    Returns:
        The resulting pose as a 6x1 numpy array.
    """
    p = np.zeros((6, 1))
    p[0:3] = point_to_array(pose.position)
    p[3:6] = quaternion_to_rotation(pose.orientation).as_euler("xyz").reshape((3, 1))

    return p
