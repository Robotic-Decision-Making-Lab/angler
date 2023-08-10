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
from typing import Any

from control_msgs.action import FollowJointTrajectory
from controllers.controller import BaseController
from rclpy.action import ActionServer
from rclpy.time import Time
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class BaseMultiDOFJointTrajectoryController(BaseController):
    def __init__(self, controller_name: str) -> None:
        super().__init__(controller_name)

        self.execute_trajectory_client = ActionServer(
            self,
            FollowJointTrajectory,
            "/angler/action/execute_cartesian_trajectory",
            self.execute_trajectory,
        )

    @abstractmethod
    def execute_trajectory(
        self, goal: FollowJointTrajectory.Goal
    ) -> FollowJointTrajectory.Result:
        ...


class Trajectory:
    def __init__(self, trajectory: MultiDOFJointTrajectory | None = None) -> None:
        if trajectory is not None:
            self.trajectory = trajectory
            self.start_time = Time(
                seconds=trajectory.header.stamp.sec,
                nanoseconds=trajectory.header.stamp.nanosec,
            )
        else:
            self.trajectory = MultiDOFJointTrajectory()
            self.start_time = Time()

        self.time_before_trajectory = Time()
        self.state_before_trajectory = MultiDOFJointTrajectoryPoint()

    def _set_point_before_trajectory_msg(
        self,
        current_time: Time,
        current_point: MultiDOFJointTrajectoryPoint,
    ) -> None:
        self.time_before_trajectory = current_time
        self.state_before_trajectory = current_point

    @staticmethod
    def merge_trajectory_with_current_state(
        current_time: Time,
        state: MultiDOFJointTrajectoryPoint,
        trajectory: MultiDOFJointTrajectory,
    ) -> Any:
        traj = Trajectory(trajectory)
        traj._set_point_before_trajectory_msg(current_time, state)

        return traj

    def update(self, trajectory: MultiDOFJointTrajectory):
        self.trajectory = trajectory
        self.start_time = Time(
            seconds=trajectory.header.stamp.sec,
            nanoseconds=trajectory.header.stamp.nanosec,
        )
        self.sampled_already = False

    def sample(
        self,
        t: Time,
        start_point: MultiDOFJointTrajectoryPoint,
        end_point: MultiDOFJointTrajectoryPoint,
    ) -> MultiDOFJointTrajectoryPoint | None:
        if len(self.trajectory.points) <= 0:
            return None

        if not self.sampled_already:
            if self.start_time.seconds_nanoseconds()[0] == 0:
                self.start_time = t
            self.sampled_already = True

        if t < self.time_before_trajectory:
            return None

        output_state = MultiDOFJointTrajectoryPoint()
        first_point_in_msg = self.trajectory.points[0]
        first_point_timestamp = self.start_time + first_point_in_msg.time_from_start

        if t < first_point_timestamp:
            if True:
                ...

    def interpolate(self):
        ...
