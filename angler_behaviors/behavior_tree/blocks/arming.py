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

import behavior_tree.context as context
import py_trees
import py_trees_ros
from components.service_clients import FromConstant as ServiceClientFromConstant
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def make_save_armed_behavior() -> py_trees.behaviour.Behaviour:
    """Save a command to arm/disarm the system to the blackboard.

    Returns:
        A ToBlackboard behavior which saves commands to arm/disarm the system to the
        blackboard.
    """
    return py_trees_ros.subscribers.ToBlackboard(
        name="ROS2BB: Arm",
        topic_name=context.TOPIC_ARM,
        topic_type=Bool,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variables={context.BB_ARMED: None},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )


def make_block_on_disarm_behavior(
    on_disarm_behavior: py_trees.behaviour.Behaviour,
    tasks: py_trees.behaviour.Behaviour,
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that blocks when the system is disarmed.

    Args:
        on_disarm_behavior: The behavior to run when a disarm is triggered.
        tasks: A behavior with the tasks to run.

    Returns:
        A Selector behavior with the disarm EternalGuard as the highest priority
        behavior and the provided tasks as second priority.
    """

    def check_disarm_on_blackboard(
        blackboard: py_trees.blackboard.Blackboard,
    ) -> bool:
        return blackboard.disarm  # type: ignore

    disarm = py_trees.decorators.EternalGuard(
        name="Disarm?",
        condition=check_disarm_on_blackboard,
        blackboard_keys={"disarm"},
        child=on_disarm_behavior,
    )

    return py_trees.composites.Selector(
        name="Block tasks on disarm", memory=False, children=[disarm, tasks]
    )


def make_arming_behavior(
    arm: bool, post_arming_behavior: py_trees.behaviour.Behaviour | None
):
    set_passthrough_mode = ServiceClientFromConstant(
        name=f"Enable PWM passthrough mode: {arm}",
        service_type=SetBool,
        service_name=context.SRV_ENABLE_PASSTHROUGH,
        service_request=Bool(data=arm),
        key_response=context.BB_PASSTHROUGH_REQUEST_RESPONSE,
    )

    arm_blue_controller = ServiceClientFromConstant(
        name=f"Arm Blue controller: {arm}",
        service_type=SetBool,
        service_name=context.SRV_ARM_BLUE,
        service_request=Bool(data=arm),
        key_response=context.BB_BLUE_ARMING_REQUEST_RESPONSE,
    )

    arm_angler_controller = ServiceClientFromConstant(
        name=f"Arm Angler controller: {arm}",
        service_type=SetBool,
        service_name=context.SRV_ARM_ANGLER,
        service_request=Bool(data=arm),
        key_response=context.BB_ANGLER_ARMING_REQUEST_RESPONSE,
    )

    # TODO(evan): Save the result / add a post arming behavior
    # TODO(evan): Wait for the result of each service call to be true

    return py_trees.composites.Sequence(
        name="Arm system" if arm else "Disarm system",
        memory=True,
        children=[set_passthrough_mode, arm_blue_controller, arm_angler_controller],
    )
