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

from abc import ABC, abstractmethod

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class WaypointPlanner(Node, ABC):
    def __init__(self) -> None:
        Node.__init__(self, "waypoint_planner")
        ABC.__init__(self)

    def get_task_plan(self, planning_context):
        ...

    @abstractmethod
    def plan(self, context):
        raise NotImplementedError("This method has not yet been implemented!")


class PreplannedWaypointPlanner(WaypointPlanner):
    def __init__(self) -> None:
        super().__init__()

    def plan(self, context):
        ...


def main_preplanned(args=None):
    """Run the pre-planned waypoint planner."""
    rclpy.init(args=args)

    node = PreplannedWaypointPlanner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
