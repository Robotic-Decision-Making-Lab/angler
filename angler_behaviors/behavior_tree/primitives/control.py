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
from moveit_msgs.srv import GetMotionPlan
from py_trees_ros.action_clients import FromBlackboard
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


def make_move_to_end_effector_pose_behavior(
    desired_pose_key: str, controller_id: str
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that moves the system to a given end-effector pose.

    Args:
        desired_pose_key: The blackboard key which holds the desired pose (saved as a
            Transform).
        controller_id: The ID of the whole-body controller to load.

    Returns:
        A behavior that moves the system to a desired end-effector pose.
    """

    def make_move_to_pose_goal(desired_pose: Transform) -> FollowJointTrajectory.Goal:
        point = MultiDOFJointTrajectoryPoint()
        point.transforms.append(desired_pose)  # type: ignore
        goal = FollowJointTrajectory.Goal()
        goal.multi_dof_trajectory.points.append(point)  # type: ignore
        return goal

    desired_ee_pose_key = "desired_end_effector_pose"

    get_desired_pose = FunctionOfBlackboardVariables(
        name="Get the desired end-effector pose",
        input_keys=[desired_pose_key],
        output_key=desired_ee_pose_key,
        function=make_move_to_pose_goal,
    )

    move_to_pose = FromBlackboard(
        name="Move to the desired end-effector pose",
        action_type=FollowJointTrajectory,
        action_name=f"/angler/{controller_id}/execute_trajectory",
        key=desired_ee_pose_key,
    )

    return py_trees.composites.Sequence(
        name="Move to end-effector pose",
        memory=True,
        children=[get_desired_pose, move_to_pose],
    )


def make_execute_multidof_trajectory_behavior(
    trajectory_key: str, controller_id: str
) -> py_trees.behaviour.Behaviour:
    """Make a behavior that executes a multi-DOF joint trajectory.

    Args:
        trajectory_key: The blackboard key at which the trajectory is being stored.
        controller_id: The ID of the controller to run.

    Returns:
        A behavior that executes a multi-DOF joint trajectory.
    """

    def make_follow_trajectory_goal(
        response: GetMotionPlan.Response,
    ) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.multi_dof_trajectory = (
            response.motion_plan_response.trajectory.multi_dof_joint_trajectory
        )
        return goal

    desired_trajectory_key = "desired_trajectory"

    get_desired_trajectory = FunctionOfBlackboardVariables(
        name="Get the desired trajectory to track",
        input_keys=[trajectory_key],
        output_key=desired_trajectory_key,
        function=make_follow_trajectory_goal,
    )

    follow_trajectory = FromBlackboard(
        name="Follow the joint trajectory",
        action_type=FollowJointTrajectory,
        action_name=f"/angler/{controller_id}/execute_trajectory",
        key=desired_trajectory_key,
    )

    return py_trees.composites.Sequence(
        name="Load and execute a multi-DOF joint trajectory",
        memory=True,
        children=[get_desired_trajectory, follow_trajectory],
    )
