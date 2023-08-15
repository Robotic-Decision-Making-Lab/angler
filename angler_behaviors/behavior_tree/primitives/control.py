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
from behavior_tree.primitives.blackboard import FunctionOfBlackboardVariables
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Transform
from py_trees_ros.action_clients import FromBlackboard
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


def make_move_to_end_effector_pose_behavior(
    desired_pose_key: str, controller_id_key: str
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that moves the system to a given end-effector pose.

    Args:
        desired_pose_key: The blackboard key which holds the desired pose (saved as a
            Transform).
        controller_id_key: The key which holds the name of the controller to use.

    Returns:
        A behavior that moves the system to a desired end-effector pose.
    """

    def make_move_to_pose_goal(desired_pose: Transform) -> FollowJointTrajectory.Goal:
        point = MultiDOFJointTrajectoryPoint()
        point.transforms.append(desired_pose)  # type: ignore
        goal = FollowJointTrajectory.Goal()
        goal.multi_dof_trajectory.points.append(point)  # type: ignore
        return goal

    get_desired_pose = FunctionOfBlackboardVariables(
        "Get the desired end-effector pose",
        [desired_pose_key],
        "desired_end_effector_pose",
        make_move_to_pose_goal,
    )

    # Get the controller ID to reconstruct the planning topic
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key=controller_id_key, access=py_trees.common.Access.READ)
    controller_id = blackboard.get(controller_id_key)

    move_to_pose = FromBlackboard(
        "Move to the desired end-effector pose",
        FollowJointTrajectory,
        f"/angler/{controller_id}/execute_trajectory",
        "desired_end_effector_pose",
    )

    return py_trees.composites.Sequence(
        "Move to end-effector pose",
        memory=True,
        children=[get_desired_pose, move_to_pose],
    )


def make_execute_multidof_trajectory_behavior(
    trajectory_key: str, controller_id_key: str
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that executes a multi-DOF joint trajectory.

    Args:
        trajectory_key: The blackboard key at which the trajectory is being stored.
        controller_id_key: The ID of the controller to run.

    Returns:
        A behavior that executes a multi-DOF joint trajectory.
    """

    def make_follow_trajectory_goal(
        trajectory: MultiDOFJointTrajectory,
    ) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.multi_dof_trajectory = trajectory
        return goal

    get_desired_trajectory = FunctionOfBlackboardVariables(
        "Get the desired trajectory to track",
        [trajectory_key],
        "desired_trajectory",
        make_follow_trajectory_goal,
    )

    # Get the controller ID to reconstruct the planning topic
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key=controller_id_key, access=py_trees.common.Access.READ)
    controller_id = blackboard.get(controller_id_key)

    follow_trajectory = FromBlackboard(
        "Follow the joint trajectory",
        FollowJointTrajectory,
        f"/angler/{controller_id}/execute_trajectory",
        "desired_trajectory",
    )

    return py_trees.composites.Sequence(
        "Load and execute a multi-DOF joint trajectory",
        memory=True,
        children=[get_desired_trajectory, follow_trajectory],
    )
