# # Copyright 2023, Evan Palmer
# #
# # Permission is hereby granted, free of charge, to any person obtaining a copy
# # of this software and associated documentation files (the "Software"), to deal
# # in the Software without restriction, including without limitation the rights
# # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# # copies of the Software, and to permit persons to whom the Software is
# # furnished to do so, subject to the following conditions:
# #
# # The above copyright notice and this permission notice shall be included in
# # all copies or substantial portions of the Software.
# #
# # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# # THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# # THE SOFTWARE.

# import os

# import kinpy
# import numpy as np
# import pytest
# import tpik.conversions as conversions
# import tpik.jacobian as J
# import xacro
# from ament_index_python import get_package_share_directory
# from geometry_msgs.msg import Quaternion


# @pytest.fixture
# def rot_map_to_base():
#     """Fixture used to generate a common map -> base_link transform.

#     Yields:
#         The resulting map -> base_link transformation.
#     """
#     q = Quaternion()

#     q.x = 4.0
#     q.y = 5.0
#     q.z = 6.0
#     q.w = 7.0

#     yield q


# @pytest.fixture
# def serial_chain():
#     """Generate a kinpy serial chain.

#     Yields:
#         The resulting serial chain from base_link -> ee_base_link
#     """
#     urdf_path = os.path.join(
#         get_package_share_directory("angler_planning"),
#         "test",
#         "resources",
#         "alpha.config.xacro",
#     )
#     doc = xacro.parse(open(urdf_path))
#     xacro.process_doc(doc)

#     yield kinpy.build_serial_chain_from_urdf(
#         doc.toxml(),  # type: ignore
#         end_link_name="ee_base_link",
#         root_link_name="base_link",
#     )


# def test_vehicle_angular_velocity_jacobian(rot_map_to_base) -> None:
#     """Test the vehicle angular velocity Jacobian calculation."""
#     rot = conversions.quaternion_to_rotation(rot_map_to_base)
#     r, p, _ = rot.as_euler("xyz")

#     sp = np.sin(p)
#     cp = np.cos(p)
#     sr = np.sin(r)
#     cr = np.cos(r)

#     # Calculate the expected Jacobian using Antonelli's Equation 2.3 from
#     # "Underwater Robots"
#     expected = np.array(
#         [[1, 0, -sp], [0, cr, cp * sr], [0, -sr, cp * cr]]  # type: ignore
#     )
#     actual = J.calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

#     assert (expected == actual).all()


# def test_manipulator_jacobian(serial_chain) -> None:
#     """Test that the manipulator Jacobian is generated from dummy joint angles.

#     We don't need to test the library's implementation; we just want to make sure
#     that we are using it correctly!
#     """
#     # This test will fail if the method throws
#     J.calculate_manipulator_jacobian(serial_chain, np.zeros((1, 4)))


# def test_vehicle_orientation_jacobian(serial_chain, rot_map_to_base) -> None:
#     """Test that the vehicle orientation Jacobian is calculated properly."""
#     rot_map_to_base = np.linalg.inv(rot_map_to_base)
#     n_joints = len(serial_chain.get_joint_parameter_names())

#     expected = np.zeros((3, 6 + n_joints))
#     expected[:, 3:6] = rot_map_to_base

#     actual = J.calculate_vehicle_orientation_jacobian(rot_map_to_base, n_joints)

#     assert (expected == actual).all()


# def test_vehicle_roll_pitch_jacobian(serial_chain, rot_map_to_base) -> None:
#     """Test that the vehicle roll-pitch jacobian is calculated correctly."""
#     n_joints = len(serial_chain.get_joint_parameter_names())

#     expected = np.zeros((2, 6 + n_joints))
#     expected[:, 3:6] = np.array(
#         [[1, 0, 0], [0, 1, 0]]
#     ) @ J.calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)

#     actual = J.calculate_vehicle_roll_pitch_jacobian(rot_map_to_base, n_joints)

#     assert (expected == actual).all()


# def test_vehicle_yaw_jacobian(serial_chain, rot_map_to_base) -> None:
#     """Test the vehicle yaw Jacobian calculation."""
#     expected = np.zeros((1, 6 + len(serial_chain.get_joint_parameter_names())))
#     expected[3:6] = np.array([0, 0, 1]) @ np.linalg.inv(
#         J.calculate_vehicle_angular_velocity_jacobian(rot_map_to_base)
#     )

#     actual = J.calculate_vehicle_yaw_jacobian(
#         rot_map_to_base, len(serial_chain.get_joint_parameter_names())
#     )
#     assert (expected == actual).all()


# def test_joint_limit_jacobian(serial_chain) -> None:
#     """Test that the joint limit task Jacobian is calculated correctly."""
#     limit_index = 0
#     num_joints = len(serial_chain.get_joint_parameter_names())

#     expected = np.zeros((1, 6 + num_joints))
#     expected[limit_index] = 1

#     actual = J.calculate_joint_limit_jacobian(limit_index, num_joints)

#     assert (expected == actual).all()
