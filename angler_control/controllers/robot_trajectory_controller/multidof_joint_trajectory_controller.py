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

from abc import abstractmethod

import rclpy
from control_msgs.action import FollowJointTrajectory
from controllers.controller import BaseController
from controllers.robot_trajectory_controller.trajectory import MultiDOFTrajectory
from rclpy.action import ActionServer, GoalResponse
from rclpy.timer import Timer


class MultiDOFJointTrajectoryController(BaseController):
    """Base interface for a MultiDOFJointTrajectory controller."""

    def __init__(self, controller_name: str) -> None:
        """Create a new multi-DOF joint trajectory controller.

        Args:
            controller_name: The unique controller name.
        """
        super().__init__(controller_name)

        self.declare_parameter("goal_tolerance", 0.2)

        self.goal_handle: FollowJointTrajectory.Goal | None = None
        self.goal_result: FollowJointTrajectory.Result | None = None
        self._preempted = False
        self._running = False
        self._timer: Timer | None = None
        self.goal_tolerance = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )

        self.execute_trajectory_client = ActionServer(
            self,
            FollowJointTrajectory,
            f"/angler/{controller_name}/execute_trajectory",
            goal_callback=self.handle_action_request_cb,
            execute_callback=self.execute_trajectory_cb,
        )

    def handle_action_request_cb(
        self, goal: FollowJointTrajectory.Goal
    ) -> GoalResponse:
        """Accept or reject the action request.

        Args:
            goal: The action goal.

        Returns:
            Whether or not the goal has been accepted.
        """
        if not self.armed:
            return GoalResponse.REJECT

        if self.goal_handle is not None:
            self._preempted = True
            return GoalResponse.ACCEPT

        return GoalResponse.ACCEPT

    @abstractmethod
    def on_update(self) -> None:
        """Return if no goal has been received."""
        if self.goal_handle is None:
            return

        # TODO(evan): execute the trajectory

    async def execute_trajectory_cb(
        self, goal: FollowJointTrajectory.Goal
    ) -> FollowJointTrajectory.Result:
        """Execute a trajectory.

        Args:
            goal: The goal with the desired trajectory to execute.

        Raises:
            RuntimeError: Unhandled state.

        Returns:
            The trajectory execution result.
        """
        # Wait for any previous goals to be cleaned up
        timer = self.create_rate(self.dt)
        while self._running and rclpy.ok():
            self.get_logger().info(
                "Waiting for previous joint trajectory goal to exit!"
            )
            timer.sleep()

        # Clean things up after the previous action
        self._preempted = False
        self._running = True
        self.goal_handle = goal
        self.goal_result = None
        self.trajectory = MultiDOFTrajectory(
            goal.multi_dof_trajectory,
            self.state.multi_dof_joint_state,
            self.get_clock().now(),
        )

        # Spin until we get a response from the controller or the goal is preempted/
        # canceled
        while (
            rclpy.ok()
            and not self._preempted
            and self._running
            and self.goal_result is None
            and not self.goal_handle.is_cancel_requested  # type: ignore
        ):
            timer.sleep()

        self._running = False

        if self._preempted:
            return self.preempt_current_goal()
        elif self.goal_handle.is_cancel_requested:  # type: ignore
            return self.cancel_current_goal()
        elif self.goal_result is not None:
            return self.goal_result

        # If we get here, something bad happened
        raise RuntimeError(
            f"An error occurred while attempting to execute the goal {goal}"
        )

    def preempt_current_goal(self) -> FollowJointTrajectory.Result:
        """Preempt the current goal with a new goal.

        Returns:
            The result of the goal that has been preempted.
        """
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = (
            "The current action was cancelled by a new incoming action"
        )
        self.goal_handle.canceled()  # type: ignore
        return result

    def cancel_current_goal(self) -> FollowJointTrajectory.Result:
        """Cancel the current goal.

        Returns:
            The result of the goal that has been cancelled.
        """
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = "The current action was cancelled by a client!"
        self.goal_handle.canceled()  # type: ignore
        return result
