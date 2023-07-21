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

import os

import numpy as np
import pytest
import tpik.conversions as conversions
import tpik.jacobian as J
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Transform


@pytest.fixture
def transform_map_to_base():
    """Fixture used to generate a common map -> base_link transform.

    Yields:
        The resulting map -> base_link transformation.
    """
    t = Transform()

    t.translation.x = 1.0
    t.translation.y = 2.0
    t.translation.z = 3.0

    t.rotation.x = 4.0
    t.rotation.y = 5.0
    t.rotation.z = 6.0
    t.rotation.w = 7.0

    yield t


@pytest.fixture
def serial_chain():
    urdf_path = os.path.join


def test_vehicle_angular_velocity_jacobian(transform_map_to_base) -> None:
    """Test the vehicle angular velocity Jacobian calculation."""
    rot = conversions.quaternion_to_rotation(transform_map_to_base.rotation)
    r, p, _ = rot.as_euler("xyz")

    sp = np.sin(p)
    cp = np.cos(p)
    sr = np.sin(r)
    cr = np.cos(r)

    # Calculate the expected Jacobian using Antonelli's Equation 2.3 from
    # "Underwater Robots"
    expected = np.array([[1, 0, -sp], [0, cr, cp * sr], [0, -sr, cp * cr]])
    actual = J.calculate_vehicle_angular_velocity_jacobian(transform_map_to_base)

    assert (expected == actual).all()


def test_vehicle_jacobian(transform_map_to_base) -> None:
    """Test the full vehicle Jacobian calculation."""
    rot = conversions.quaternion_to_rotation(transform_map_to_base.rotation)

    expected = np.zeros((6, 6))

    # Calculate the expected Jacobian using Antonelli's Equation 2.19 from
    # "Underwater Robots"
    expected[0:3, 0:3] = rot.as_matrix()
    expected[3:6, 3:6] = J.calculate_vehicle_angular_velocity_jacobian(
        transform_map_to_base
    )

    actual = J.calculate_vehicle_jacobian(transform_map_to_base)

    assert (expected == actual).all()


def test_manipulator_jacobian() -> None:
    ...


def test_uvms_jacobian() -> None:
    ...


def test_vehicle_roll_pitch_jacobian() -> None:
    ...


def test_vehicle_yaw_jacobian() -> None:
    ...


def test_joint_limit_jacobian() -> None:
    ...


def test_vehicle_manipulator_collision_avoidance_jacobian() -> None:
    ...
