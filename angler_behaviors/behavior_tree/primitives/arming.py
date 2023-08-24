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
from behavior_tree.primitives.blackboard import (
    FunctionOfBlackboardVariables,
    ToBlackboardNonBlocking,
)
from behavior_tree.primitives.service_clients import FromConstant
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def make_save_armed_behavior(arm_system_key: str) -> py_trees.behaviour.Behaviour:
    """Save a command to arm/disarm the system to the blackboard.

    Returns:
        A ToBlackboard behavior which saves commands to arm/disarm the system to the
        blackboard.
    """
    return ToBlackboardNonBlocking(
        name="ROS2BB: Arm system",
        topic_name="/angler/cmd/arm_autonomy",
        topic_type=Bool,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={arm_system_key: "data"},
        initialise_variables={arm_system_key: False},
        clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS,
    )


def make_subsystem_arming_behavior(
    arm: bool, subsystem_name: str, arming_topic: str
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that arms/disarms a subsystem (e.g., a whole-body controller).

    Args:
        arm: True if arming; False if disarming.
        subsystem_name: The name of the sub-system to arm.
        arming_topic: The topic over which to send the arming request.

    Returns:
        A behavior that arms/disarms a subsystem.
    """

    def check_arming_success(response: SetBool.Response) -> bool:
        return response.success

    arm_response_key = "arm_request_response"
    arm_result_key = "arm_request_result"

    arm_request = FromConstant(
        name=f"{subsystem_name}: send arming request",
        service_type=SetBool,
        service_name=arming_topic,
        service_request=SetBool.Request(data=arm),
        key_response=arm_response_key,
    )
    transform_response = FunctionOfBlackboardVariables(
        name=f"{subsystem_name}: transform arming response to bool",
        input_keys=[arm_response_key],
        output_key=arm_result_key,
        function=check_arming_success,
    )
    check_response = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"{subsystem_name}: verify response",
        check=py_trees.common.ComparisonExpression(
            variable=arm_result_key,
            value=True,
            operator=operator.eq,
        ),
    )

    return py_trees.composites.Sequence(
        name=f"{subsystem_name}: Arming" if arm else ": Disarming",
        memory=True,
        children=[arm_request, transform_response, check_response],
    )


def make_system_arming_behavior(arm: bool, armed_key: str, use_passthrough_mode: bool):
    """Create a behavior that arms/disarms the system.

    The full system arming sequence includes:
    1. Enabling/disabling PWM passthrough mode (if configured)
    2. Arming/disarming the Blue controller
    3. Arming/disarming the Angler controller

    Args:
        arm: Set to `True` to arm the system; set to `False` to disarm.
        armed_key: The key at which to store the resulting arming status.
        use_passthrough_mode: Use the Blue PWM passthrough mode. Take care when using
            this mode. All safety checks onboard the system will be disabled.
            Furthermore, make sure to leave PWM passthrough mode before shutdown, or
            the system parameters will not be restored.

    Returns:
        A behavior that runs the full system arming sequence.
    """
    set_armed_state = py_trees.behaviours.SetBlackboardVariable(
        name="Set the current arming status",
        variable_name=armed_key,
        variable_value=arm,
        overwrite=True,
    )

    # Put everything together
    behaviors = [
        make_subsystem_arming_behavior(arm, "Blue Controller", "/blue/cmd/arm"),
        make_subsystem_arming_behavior(arm, "Angler Controller", "/angler/cmd/arm"),
        set_armed_state,
    ]

    if use_passthrough_mode:
        behaviors.insert(
            0,
            make_subsystem_arming_behavior(
                arm, "Passthrough Mode", "/blue/cmd/enable_passthrough"
            ),
        )

    check_already_disarmed = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Already armed?" if arm else "Already disarmed?",
        check=py_trees.common.ComparisonExpression(
            variable=armed_key, value=arm, operator=operator.eq
        ),
    )
    do_arming = py_trees.composites.Sequence(
        name="Enable system autonomy" if arm else "Disable system autonomy",
        memory=True,
        children=behaviors,  # type: ignore
    )

    return py_trees.composites.Selector(
        "Don't do what has already been done!",
        memory=False,
        children=[check_already_disarmed, do_arming],
    )


def make_block_on_disarm_behavior(
    arm_system_key: str,
    armed_key: str,
    tasks: py_trees.behaviour.Behaviour,
    on_disarm_behavior: py_trees.behaviour.Behaviour | None = None,
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that blocks when the system is disarmed.

    Args:
        arm_system_key: The key at which the arm system flag is stored.
        armed_key: The key at which to store the resulting arming status.
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
        # We want to stop when a user issues a disarm command
        return not blackboard.get(arm_system_key)

    # Default to disarming passthrough mode just in case
    disarming = make_system_arming_behavior(
        False, armed_key=armed_key, use_passthrough_mode=True
    )

    behaviors = [disarming]

    if on_disarm_behavior is not None:
        behaviors.append(on_disarm_behavior)  # type: ignore

    on_disarm = py_trees.composites.Sequence(
        name="Execute disarm sequence",
        memory=True,
        children=behaviors,  # type: ignore
    )

    disarm = py_trees.decorators.EternalGuard(
        name="Disarm?",
        condition=check_disarm_on_blackboard,
        blackboard_keys=[arm_system_key],
        child=on_disarm,
    )

    return py_trees.composites.Selector(
        name="Block tasks on disarm", memory=False, children=[disarm, tasks]
    )
