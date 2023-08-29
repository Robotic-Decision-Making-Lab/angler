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

import py_trees
import py_trees_ros
from std_msgs.msg import Float64MultiArray


def make_alpha_jaws_velocity_control_behavior(
    open_gripper: bool, rate: float
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that opens/closes the Reach Alpha jaws.

    Args:
        open_gripper: Whether or not to open the gripper.
        rate: The velocity that the gripper should open at (mm/s).

    Returns:
        A behavior that controls the Reach Alpha jaws using the
            forward_velocity_controller.
    """
    gripper_command_key = "gripper_command"

    if open_gripper:
        rate = abs(rate)
    else:
        rate = -abs(rate)

    set_command = py_trees.behaviours.SetBlackboardVariable(
        name="Set gripper command",
        variable_name=gripper_command_key,
        variable_value=Float64MultiArray(data=[rate, 0.0, 0.0, 0.0, 0.0]),
        overwrite=True,
    )
    publish_command = py_trees_ros.publishers.FromBlackboard(
        name="Publish gripper command",
        topic_name="/forward_velocity_controller/commands",
        topic_type=Float64MultiArray,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable=gripper_command_key,
    )

    return py_trees.composites.Sequence(
        name="Command the gripper", memory=True, children=[set_command, publish_command]
    )
