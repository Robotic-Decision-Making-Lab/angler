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
from behavior_tree.primitives.blackboard import FunctionOfBlackboardVariables
from behavior_tree.primitives.service_clients import FromBlackboard
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetMotionPlan
from rclpy.qos import qos_profile_sensor_data


def make_save_robot_state_behavior(
    robot_state_key: str,
) -> py_trees.behaviour.Behaviour:
    """Save the current robot state.

    Returns:
        A ToBlackboard behavior which saves the robot state.
    """
    return py_trees_ros.subscribers.ToBlackboard(
        name="ROS2BB: Robot state",
        topic_name="/angler/state",
        topic_type=RobotState,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={robot_state_key: None},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )


def make_high_level_planning_behavior(
    robot_state_key: str, planning_result_key: str, planner_id: str
) -> py_trees.behaviour.Behaviour:
    """Create a high-level planning behavior.

    Args:
        robot_state_key: The key at which the robot state is being stored.
        planning_result_key: The key at which the planning result is saved.
        planner_id: The ID of the planner to perform high-level planning with.

    Returns:
        Behavior that creates a high-level mission plan and saves the result to the
        blackboard.
    """

    def make_planning_request(state: RobotState) -> GetMotionPlan.Request:
        request = GetMotionPlan.Request()
        request.motion_plan_request.start_state = state
        request.motion_plan_request.planner_id = planner_id
        return request

    planner_request_key = "high_level_planner_request"

    plan_request_behavior = FunctionOfBlackboardVariables(
        name="Make high-level planner request",
        input_keys=[robot_state_key],
        output_key=planner_request_key,
        function=make_planning_request,
    )

    plan_behavior = FromBlackboard(
        name="Plan a high-level mission",
        service_type=GetMotionPlan,
        service_name=f"/angler/{planner_id}/plan",
        key_request=planner_request_key,
        key_response=planning_result_key,
    )

    return py_trees.composites.Sequence(
        name="Get high-level plan",
        memory=True,
        children=[plan_request_behavior, plan_behavior],
    )
