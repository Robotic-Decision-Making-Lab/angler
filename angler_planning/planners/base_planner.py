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

from abc import ABC, abstractmethod

from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node


class Planner(Node, ABC):
    """Base class for a planner."""

    def __init__(self, node_name: str) -> None:
        """Create a new planner.

        Args:
            node_name: The name of the ROS node.
        """
        super().__init__(node_name)
        Node.__init__(self, node_name)
        ABC.__init__(self)

        self.planning_srv = self.create_service(
            GetMotionPlan, f"~/angler/{node_name}/get_motion_plan", self.plan
        )

    @abstractmethod
    def plan(
        self, request: GetMotionPlan.Request, response: GetMotionPlan.Response
    ) -> GetMotionPlan.Response:
        """Get a motion plan for the robot to follow.

        Args:
            request: The motion plan request.
            response: The planning response.

        Raises:
            NotImplementedError: This method hasn't been implemented.

        Returns:
            The planning response.
        """
        raise NotImplementedError("This method has not yet been implemented!")
