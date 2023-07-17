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


def calculate_vehicle_angular_velocity_jacobian(
    angular_velocity: np.ndarray,
) -> np.ndarray:
    """underwater robots pg 25, eq 2.3"""
    roll, pitch, _ = angular_velocity

    return np.array(
        [
            [1, 0, np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch) * np.sin(roll)],
            [0 - np.sin(roll), np.cos(pitch * np.cos(roll))],
        ]
    )


def calculate_vehicle_jacobian(
    orientation: np.ndarray, angular_velocity_jacobian: np.ndarray
) -> np.ndarray:
    """underwater robots pg 31, eq 2.19"""
    return np.array(
        [[orientation, np.zeros((3, 3))], [np.zeros((3, 3)), angular_velocity_jacobian]]
    )


def calculate_manipulator_jacobian(joint_angles: np.ndarray) -> np.ndarray:
    ...
