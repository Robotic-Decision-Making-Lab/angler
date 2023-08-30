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

import numpy as np
from geometry_msgs.msg import Point, Quaternion, Transform, Twist, Vector3
from scipy.spatial.transform import Rotation as R


def tf_to_numpy(tf: Transform) -> np.ndarray:
    """Convert a Transform message to a numpy array.

    Args:
        tf: The transform to convert.

    Returns:
        The resulting numpy array.
    """
    return np.array(
        [
            tf.translation.x,
            tf.translation.y,
            tf.translation.z,
            *R.from_quat(
                [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
            ).as_euler("xyz"),
        ]
    )


def twist_to_numpy(twist: Twist) -> np.ndarray:
    """Convert a Twist message to a numpy array.

    Args:
        twist: The twist to convert.

    Returns:
        The resulting numpy array.
    """
    return np.array(
        [
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.z,
        ]
    )


def quaternion_to_numpy(quat: Quaternion):
    """Convert a Quaternion message into a numpy array.

    Args:
        quat: The quaternion to convert.

    Returns:
        The resulting numpy array.
    """
    return np.array([quat.x, quat.y, quat.z, quat.w])


def quaternion_to_rotation(quat: Quaternion) -> R:
    """Convert a Quaternion message into a SciPy Rotation.

    Args:
        quat: The Quaternion message to convert.

    Returns:
        The resulting SciPy Rotation.
    """
    return R.from_quat([quat.x, quat.y, quat.z, quat.w])


def point_to_numpy(point: Point | Vector3) -> np.ndarray:
    """Convert a Point message into a numpy array.

    Args:
        point: The Point message to convert.

    Returns:
        The resulting point as a 3x1 numpy array.
    """
    return np.array([point.x, point.y, point.z]).reshape((3, 1))
