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
import rclpy
from rclpy.node import Node


def calculate_nullspace(augmented_jacobian: np.ndarray) -> np.ndarray:
    """Calculate the nullspace of the augmented Jacobian.

    Args:
        augmented_jacobian: The augmented Jacobian whose nullspace will be projected
            into.

    Returns:
        The nullspace of the augmented Jacobian.
    """
    return (
        np.eye(augmented_jacobian.shape[0], augmented_jacobian.shape[1])
        - np.linalg.pinv(augmented_jacobian) @ augmented_jacobian
    )


def construct_augmented_jacobian(jacobians: list[np.ndarray]) -> np.ndarray:
    """Construct an augmented Jacobian matrix.

    The augmented Jacobian matrix is given as:

    J_i^A = [J_1, J_2, ..., J_i]^T

    Args:
        jacobians: The Jacobian matrices which should be used to construct the
            augmented Jacobian.

    Returns:
        The resulting augmented Jacobian.
    """
    return np.vstack(tuple(jacobians))


class TPIK(Node):
    """Task-priority inverse kinematic (TPIK) controller.

    The TPIK controller is responsible for calculating kinematically feasible system
    velocities subject to equality and set-based constraints.
    """

    def __init__(self) -> None:
        """Create a new TPIK node."""
        super().__init__("tpik")


def main(args: list[str] | None = None):
    """Run the TPIK controller."""
    rclpy.init(args=args)

    node = TPIK()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
