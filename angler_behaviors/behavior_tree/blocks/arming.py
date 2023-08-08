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

import operator

import py_trees
import py_trees_ros
from components.blackboard import FunctionOfBlackboardVariables
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
        topic_name="/angler/cmd/arm_autonomy",
        topic_type=Bool,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variables={"armed": None},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )


def make_block_on_disarm_behavior(
    tasks: py_trees.behaviour.Behaviour,
    on_disarm_behavior: py_trees.behaviour.Behaviour | None = None,
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that blocks when the system is disarmed.

    Args:
        tasks: A behavior with the tasks to run.
        on_disarm_behavior: An optional behavior to run when a disarm is triggered. This
            will be executed as a Sequence behavior following the disarm behavior.

    Returns:
        A Selector behavior with the disarm EternalGuard as the highest priority
        behavior and the provided tasks as second priority.
    """

    def check_disarm_on_blackboard(
        blackboard: py_trees.blackboard.Blackboard,
    ) -> bool:
        return blackboard.disarm  # type: ignore

    # Default to disarming passthrough mode just in case
    disarming = make_arming_behavior(False, use_passthrough_mode=True)

    behaviors = [disarming]

    if on_disarm_behavior is not None:
        behaviors.append(on_disarm_behavior)  # type: ignore

    on_disarm = py_trees.composites.Sequence(
        name="Execute disarm sequence",
        memory=True,
        children=[behaviors],  # type: ignore
    )

    disarm = py_trees.decorators.EternalGuard(
        name="Disarm?",
        condition=check_disarm_on_blackboard,
        blackboard_keys={"disarm"},
        child=on_disarm,
    )

    return py_trees.composites.Selector(
        name="Block tasks on disarm", memory=False, children=[disarm, tasks]
    )


def make_arming_behavior(arm: bool, use_passthrough_mode: bool):
    """Create a behavior that arms/disarms the system.

    The full system arming sequence includes:
    1. Enabling/disabling PWM passthrough mode (if configured)
    2. Arming/disarming the Blue controller
    3. Arming/disarming the Angler controller

    Args:
        arm: Set to `True` to arm the system; set to `False` to disarm.
        use_passthrough_mode: Use the Blue PWM passthrough mode. Take care when using
            this mode. All safety checks onboard the system will be disabled.
            Furthermore, make sure to leave PWM passthrough mode before shutdown, or
            the system parameters will not be restored.

    Returns:
        A behavior that runs the full system arming sequence.
    """
    # For each task in the arming sequence we construct a sequence node that performs
    # the following steps:
    # 1. Sends a request to arm/disarm
    # 2. Transforms the service response into a bool
    # 3. Checks whether the response is true (the service completed successfully)

    def check_arming_success(response: SetBool.Response) -> bool:
        return response.success

    # Set PWM Passthrough mode (this only gets added to the resulting behavior if the
    # flag is set to true)
    set_passthrough_mode_request = ServiceClientFromConstant(
        name="Send PWM passthrough mode change request",
        service_type=SetBool,
        service_name="/blue/cmd/enable_passthrough",
        service_request=SetBool.Request(data=arm),
        key_response="passthrough_request_response",
    )
    transform_set_passthrough_mode_response = FunctionOfBlackboardVariables(
        name="Transform passthrough mode response to bool",
        input_keys=["passthrough_request_response"],
        output_key="passthrough_request_result",
        function=check_arming_success,
    )
    check_set_passthrough_mode_result = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="Verify that the passthrough mode change request succeeded",
            check=py_trees.common.ComparisonExpression(
                variable="passthrough_request_result",
                value=True,
                operator=operator.eq,
            ),
        )
    )
    set_passthrough_mode = py_trees.composites.Sequence(
        name="PWM passthrough mode " + ("enabled" if arm else "disabled"),
        memory=True,
        children=[
            set_passthrough_mode_request,
            transform_set_passthrough_mode_response,
            check_set_passthrough_mode_result,
        ],
    )

    # Arm/disarm the Blue controller
    arm_blue_controller_request = ServiceClientFromConstant(
        name="Send Blue controller arming/disarming request",
        service_type=SetBool,
        service_name="/blue/cmd/arm",
        service_request=SetBool.Request(data=arm),
        key_response="blue_arming_request_response",
    )
    transform_blue_controller_arming_response = FunctionOfBlackboardVariables(
        name="Transform Blue controller arming/disarming response to bool",
        input_keys=["blue_arming_request_response"],
        output_key="blue_arming_request_result",
        function=check_arming_success,
    )
    check_blue_arming_result = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify that the Blue controller arming/disarming request succeeded",
        check=py_trees.common.ComparisonExpression(
            variable="blue_arming_request_result",
            value=True,
            operator=operator.eq,
        ),
    )
    arm_blue_controller = py_trees.composites.Sequence(
        name=("Arm" if arm else "Disarm") + " Blue controller",
        memory=True,
        children=[
            arm_blue_controller_request,
            transform_blue_controller_arming_response,
            check_blue_arming_result,
        ],
    )

    # Arm/disarm the Angler controller
    arm_angler_controller_request = ServiceClientFromConstant(
        name="Send Angler arming/disarming request",
        service_type=SetBool,
        service_name="/angler/cmd/arm",
        service_request=SetBool.Request(data=arm),
        key_response="angler_arming_request_response",
    )
    transform_angler_controller_arming_response = FunctionOfBlackboardVariables(
        name="Transform Angler controller arming/disarming response to bool",
        input_keys=["angler_arming_request_response"],
        output_key="angler_arming_request_result",
        function=check_arming_success,
    )
    check_angler_arming_result = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify that the Angler controller arming/disarming request succeeded",
        check=py_trees.common.ComparisonExpression(
            variable="angler_arming_request_result",
            value=True,
            operator=operator.eq,
        ),
    )
    arm_angler_controller = py_trees.composites.Sequence(
        name=("Arm" if arm else "Disarm") + " Angler controller",
        memory=True,
        children=[
            arm_angler_controller_request,
            transform_angler_controller_arming_response,
            check_angler_arming_result,
        ],
    )

    # Now put everything together
    behaviors = [arm_blue_controller, arm_angler_controller]

    if use_passthrough_mode:
        behaviors.insert(0, set_passthrough_mode)

    return py_trees.composites.Sequence(
        name="Arm system" if arm else "Disarm system",
        memory=True,
        children=behaviors,  # type: ignore
    )
