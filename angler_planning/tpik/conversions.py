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
