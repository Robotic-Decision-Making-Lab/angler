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
from behavior_tree.primitives.control import make_execute_multidof_trajectory_behavior
from behavior_tree.primitives.planning import make_high_level_planning_behavior


def make_save_start_mission_behavior(
    start_mission_key: str,
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that saves the 'start mission' flag to the blackboard.

    Args:
        start_mission_key: The key at which the flag should be saved.

    Returns:
        A behavior that saves the key to trigger arming.
    """
    return py_trees_ros.subscribers.EventToBlackboard(
        name="ROS2BB: Start mission",
        topic_name="/angler/cmd/start_mission",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name=start_mission_key,
    )


def make_execute_mission_behavior(
    start_mission_key: str,
    robot_state_key: str,
    planner_id: str,
    controller_id: str,
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that sets up the system prior to beginning a mission.

    Args:
        start_mission_key: The key at which the signal indicating that a mission should
            start is stored.
        robot_state_key: The key at which the robot state is stored.
        planner_id: The key at which the high-level planner ID is stored.
        controller_id: The key at which the joint trajectory controller ID is
            stored.

    Returns:
        A system setup behavior.
    """
    # Start by checking whether or not to start the mission
    check_start_mission = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Start the mission?",
        check=py_trees.common.ComparisonExpression(
            variable=start_mission_key, value=True, operator=operator.eq
        ),
    )

    get_mission_plan = make_high_level_planning_behavior(
        robot_state_key=robot_state_key,
        planner_id=planner_id,
        planning_result_key="planning_result",
    )

    execute_mission = make_execute_multidof_trajectory_behavior(
        trajectory_key="planning_result", controller_id=controller_id
    )

    return py_trees.composites.Sequence(
        name="Execute mission",
        memory=True,
        children=[
            check_start_mission,
            get_mission_plan,
            execute_mission,
        ],
    )
