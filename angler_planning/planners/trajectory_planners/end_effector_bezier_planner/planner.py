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

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Transform
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from planners.base_planner import Planner
from pydrake.trajectories import BezierCurve
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class EndEffectorBezierPlanner(Planner):
    """End effector Bezier curve trajectory planner."""

    def __init__(self) -> None:
        """Create a new planning interface."""
        super().__init__("end_effector_bezier_planner")

        self.declare_parameter("steps", 50)
        self.steps = self.get_parameter("steps").get_parameter_value().integer_value

    def _create_multi_dof_joint_trajectory_point(
        self, point: np.ndarray
    ) -> MultiDOFJointTrajectoryPoint:
        """Create a MultiDOFJointTrajectoryPoint message from a trajectory point.

        Args:
            point: The trajectory point that should be used to construct the message.

        Returns:
            A MultiDOFJointTrajectoryPoint message.
        """
        msg = MultiDOFJointTrajectoryPoint()

        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = point

        msg.transforms.append(tf)  # type: ignore

        return msg

    def plan(
        self, request: GetMotionPlan.Request, response: GetMotionPlan.Response
    ) -> GetMotionPlan.Response:
        """Plan an end-effector trajectory using Bezier curves.

        Args:
            request: The planning request. This should include the control points for
                the end-effector.
            response: The resulting Bezier curve, discretized into a joint trajectory.

        Returns:
            The resulting Bezier curve planning response.
        """
        # Keep track of the planning time
        start_t = time.time()

        # Get the control points
        control_points = [
            np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]).reshape(
                (3, 1)
            )
            for p in request.motion_plan_request.reference_trajectories[  # type: ignore
                0
            ].cartesian_trajectory.points
        ]
        # Convert the control points to a single array with the columns denoting the
        # desired positions
        control_points_ar = np.hstack(control_points)

        # Construct a Bezier curve from the control points
        curve = BezierCurve(0, 1, control_points_ar)

        # Discretize the Bezier curve using the step size
        trajectory = [curve.value((1 / self.steps) * k) for k in range(self.steps)]

        # Calculate the planning time
        planning_time = time.time() - start_t

        # Convert the trajectory array into a message
        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.frame_id = "map"
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.points = [
            self._create_multi_dof_joint_trajectory_point(p) for p in trajectory
        ]

        # Update the response
        response.motion_plan_response.trajectory.multi_dof_joint_trajectory = traj_msg
        response.motion_plan_response.planning_time = planning_time
        response.motion_plan_response.error_code = MoveItErrorCodes.SUCCESS
        response.motion_plan_response.group_name = (
            request.motion_plan_request.group_name
        )

        return response


def main(args: list[str] | None = None):
    """Run the pre-planned waypoint planner."""
    rclpy.init(args=args)

    node = EndEffectorBezierPlanner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
