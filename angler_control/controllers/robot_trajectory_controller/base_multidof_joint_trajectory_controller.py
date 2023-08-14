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

import threading
from abc import abstractmethod

import controllers.robot_trajectory_controller.utils as controller_utils
import numpy as np
from control_msgs.action import FollowJointTrajectory
from controllers.controller import BaseController
from controllers.robot_trajectory_controller.trajectory import MultiDOFTrajectory
from geometry_msgs.msg import Quaternion
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.time import Duration
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


class BaseMultiDOFJointTrajectoryController(BaseController):
    """Base interface for a MultiDOFJointTrajectory controller."""

    def __init__(self, controller_name: str) -> None:
        """Create a new multi-DOF joint trajectory controller.

        Args:
            controller_name: The unique controller name.
        """
        super().__init__(controller_name)

        self.declare_parameters(
            namespace="",
            parameters=[  # type: ignore
                ("goal_position_tolerance", 0.2),
                ("goal_orientation_tolerance", 0.1),
                ("goal_linear_velocity_tolerance", np.inf),
                ("goal_angular_velocity_tolerance", np.inf),
                ("goal_time_tolerance", 10.0),
            ],
        )

        # Keep track of whether or not a goal is running/preempted to enable transition
        # between preempted goals
        self.current_uid = 0
        self.execute_trajectory_lock = threading.Lock()
        self._running = False

        # Store a command for the child to execute
        self.command: MultiDOFJointTrajectoryPoint | None = None

        # Get the tolerances for the controller
        self.goal_position_tolerance = (
            self.get_parameter("goal_position_tolerance")
            .get_parameter_value()
            .double_value
        )
        self.goal_orientation_tolerance = (
            self.get_parameter("goal_orientation_tolerance")
            .get_parameter_value()
            .double_value
        )
        self.goal_linear_velocity_tolerance = (
            self.get_parameter("goal_linear_velocity_tolerance")
            .get_parameter_value()
            .double_value
        )
        self.goal_angular_velocity_tolerance = (
            self.get_parameter("goal_angular_velocity_tolerance")
            .get_parameter_value()
            .double_value
        )
        self.goal_time_tolerance = Duration(
            seconds=self.get_parameter("goal_time_tolerance")  # type: ignore
            .get_parameter_value()
            .double_value
        )

        self.execute_trajectory_client = ActionServer(
            self,
            FollowJointTrajectory,
            f"/angler/{controller_name}/execute_trajectory",
            goal_callback=self.handle_action_request_cb,
            execute_callback=self.execute_trajectory_cb,
        )

    def destroy_node(self) -> None:
        """Shutdown the node and all clients."""
        self.execute_trajectory_client.destroy()
        return super().destroy_node()

    @property
    @abstractmethod
    def joint_state(self) -> MultiDOFJointTrajectoryPoint:
        """Get the current joint state to control.

        Returns:
            The joint state.
        """
        ...

    def handle_action_request_cb(self, goal_handle: ServerGoalHandle) -> GoalResponse:
        """Accept or reject the action request.

        Args:
            goal_handle: The action goal.

        Returns:
            Whether or not the goal has been accepted.
        """
        if not self.armed:
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def check_joint_at_goal(
        self,
        current_state: MultiDOFJointTrajectoryPoint,
        desired_state: MultiDOFJointTrajectoryPoint,
        index: int,
    ) -> bool:
        """Check whether or not the joint is at its goal state.

        Args:
            current_state: The current joint state.
            desired_state: The desired joint state.
            index: The joint index.

        Returns:
            Whether or not the joint is at its goal.
        """
        current_pos = controller_utils.convert_tf_to_array(
            current_state.transforms[index]  # type: ignore
        )
        desired_pos = controller_utils.convert_tf_to_array(
            desired_state.transforms[index]  # type: ignore
        )

        position_at_goal = (
            np.linalg.norm(desired_pos[:3] - current_pos[:3])
            < self.goal_position_tolerance
        )

        def quat_to_array(quat: Quaternion):
            return np.array([quat.x, quat.y, quat.z, quat.w])

        current_rot = quat_to_array(
            current_state.transforms[index].rotation  # type: ignore
        )
        desired_rot = quat_to_array(
            desired_state.transforms[index].rotation  # type: ignore
        )

        error_eps = (
            current_rot[3] * desired_rot[:3]
            - desired_rot[3] * current_rot[:3]
            + np.cross(current_rot[:3], desired_rot[:3])
        )

        rotation_at_goal = (
            np.linalg.norm(np.hstack((error_eps))) < self.goal_orientation_tolerance
        )

        if (
            self.trajectory.trajectory.points[0].velocities  # type: ignore
            and self.trajectory.trajectory.points[-1].velocities  # type: ignore
        ):
            current_vel = controller_utils.convert_twist_to_array(
                current_state.velocities[index]  # type: ignore
            )
            desired_vel = controller_utils.convert_twist_to_array(
                desired_state.velocities[index]  # type: ignore
            )

            vel_error = np.linalg.norm(desired_vel - current_vel)  # type: ignore

            vel_at_goal = (
                vel_error[:3] < self.goal_linear_velocity_tolerance
                and vel_error[3:] < self.goal_angular_velocity_tolerance
            )

            return position_at_goal and rotation_at_goal and vel_at_goal

        return position_at_goal and rotation_at_goal

    def on_update(self) -> None:
        """Return if no goal has been received."""
        if not self._running:
            return

        sample = self.trajectory.sample(self.get_clock().now())

        if sample is not None:
            self.command = sample
        else:
            self.get_logger().info(
                f"Failed to sample trajectory at time {self.get_clock().now()}"
            )

    async def execute_trajectory_cb(
        self, goal_handle: ServerGoalHandle
    ) -> FollowJointTrajectory.Result:
        """Execute a joint trajectory.

        Args:
            goal_handle: The desired trajectory to track.

        Returns:
            The result of the trajectory execution.
        """
        # Keep track of the current goal UID to handle preemption
        uid = self.current_uid + 1
        self.current_uid += 1

        result = FollowJointTrajectory.Result()

        rate = self.create_rate(1 / self.dt)

        with self.execute_trajectory_lock:
            # Update the new trajectory
            self.trajectory = MultiDOFTrajectory(
                goal_handle.request.multi_dof_trajectory,
                self.joint_state,
                self.get_clock().now(),
            )
            self._running = True

            while not all(
                [
                    self.check_joint_at_goal(
                        self.joint_state,
                        self.trajectory.trajectory.points[-1],  # type: ignore
                        i,
                    )
                    for i in range(len(self.joint_state.transforms))
                ]
            ):
                # The goal has been flagged as inactive
                if not goal_handle.is_active:
                    self._running = False
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = "Goal aborted!"
                    goal_handle.abort()
                    return result

                end_goal_time = controller_utils.add_ros_time_duration_msg(
                    self.trajectory.starting_time,
                    self.trajectory.trajectory.points[-1].time_from_start,  # type: ignore # noqa
                )

                # Failed to get to the goal waypoint in the tolerance time
                if self.get_clock().now() - end_goal_time > self.goal_time_tolerance:
                    self._running = False
                    result.error_code = (
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    )
                    result.error_string = (
                        "Failed to complete the goal within the provided time"
                        f" tolerance {self.goal_time_tolerance}"
                    )
                    goal_handle.abort()
                    return result

                # Goal has been cancelled
                if goal_handle.is_cancel_requested:
                    self._running = False
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = (
                        "The current action was cancelled by a client!"
                    )
                    goal_handle.canceled()
                    return result

                # Goal has been preempted by another goal
                if uid != self.current_uid:
                    self._running = False
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = (
                        "The current action was cancelled by a new incoming action"
                    )
                    goal_handle.abort()
                    return result

                rate.sleep()

            # The goal was reached within all tolerances
            self._running = False
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "Successfully executed the provided trajectory"
            goal_handle.succeed()

            return result
