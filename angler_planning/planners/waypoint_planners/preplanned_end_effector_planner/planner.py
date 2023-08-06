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

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from planners.base_planner import Planner
from planners.waypoint_planners.preplanned_end_effector_planner.trajectories.trajectory_library import (  # noqa
    TrajectoryLibrary as tl,
)


class PrePlannedEndEffectorPlanner(Planner):
    """Planning interface for loading pre-planned end-effector trajectories."""

    def __init__(self) -> None:
        """Create a new planning interface."""
        super().__init__("preplanned_end_effector_waypoint_planner")

        self.declare_parameter("trajectory_name", "")
        self.declare_parameter(
            "library_path",
            os.path.join(
                get_package_share_directory("angler_planning"),
                "trajectories",
                "library",
            ),
        )

        trajectory_name = (
            self.get_parameter("trajectory_name").get_parameter_value().string_value
        )

        if trajectory_name == "":
            raise ValueError("Trajectory name not provided.")

        library_path = (
            self.get_parameter("library_path").get_parameter_value().string_value
        )

        # Load the trajectory library
        tl.load_library_from_path(library_path)
        self.trajectory = tl.select_trajectory(trajectory_name)

        self.get_logger().info(f"Successfully loaded trajectory '{trajectory_name}'")

    def plan(
        self, request: GetMotionPlan.Request, response: GetMotionPlan.Response
    ) -> GetMotionPlan.Response:
        """Create a motion plan from the pre-planned trajectory.

        Args:
            request: The motion plan request.
            response: The planning response.

        Returns:
            The planning response.
        """
        # Get the motion plan response
        response.motion_plan_response.trajectory = self.trajectory
        response.motion_plan_response.planning_time = 0.0
        response.motion_plan_response.error_code = MoveItErrorCodes.SUCCESS
        response.motion_plan_response.group_name = (
            request.motion_plan_request.group_name
        )

        return response


def main(args: list[str] | None = None):
    """Run the pre-planned waypoint planner."""
    rclpy.init(args=args)

    node = PrePlannedEndEffectorPlanner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
