# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from moveit_msgs.srv import GetMotionPlan
from planners.base_planner import Planner

from angler_control.controllers.robot_trajectory_controller.trajectory import (
    MultiDOFTrajectory,
)


class SplineTrajectoryPlanner(Planner):
    def __init__(self) -> None:
        super().__init__("spline_trajectory_planner")

        self.declare_parameter("max_distance", 0.3)

    def plan(
        self, request: GetMotionPlan.Request, response: GetMotionPlan.Response
    ) -> GetMotionPlan.Response:
        # TODO: Convert the GenericTrajectory in to a MultiDOFJointTrajectory
        # TODO: create a new trajectory interface
        # TODO: get the distance between the current state and the desired state
        # TODO: check the distance to the next waypoint
        # TODO: Clamp error circle radius
        # TODO: construct the error circle
        # TODO: find intersection between error circle and trajectory
        # TODO: Interpolate between the control points up to the next waypoint
        # TODO: Sample the new trajectory to a given density
        # TODO: Update the trajectory segment
