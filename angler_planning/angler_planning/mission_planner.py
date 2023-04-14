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
from abc import ABC, abstractmethod

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node

from angler_msgs.action import PlanMission
from angler_msgs.msg import Waypoint
from angler_planning.missions.loader import PrePlannedMissionLoader


class MissionPlanner(Node, ABC):
    """Base class for a high-level mission planner."""

    def __init__(self, node_name: str = "mission_planner") -> None:
        """Create a new mission planner."""
        Node.__init__(self, node_name)
        ABC.__init__(self)

        # Instantiate an action server to use for querying a mission
        self._planning_server = ActionServer(
            self, PlanMission, "angler/plan/mission", self._get_task_plan_cb
        )

    def destroy_node(self) -> bool:
        """Destroy the node.

        Returns:
            bool: Whether or not the node was successfully destroyed.
        """
        self._planning_server.destroy()
        return super().destroy_node()

    def _get_task_plan_cb(self, planning_request: PlanMission) -> PlanMission.Result:
        """Proxy the request to the planner and send the result to the client.

        Args:
            planning_request (PlanMission): The planning request.

        Returns:
            PlanMission.Result: The planning algorithm's result.
        """
        result = planning_request.Result()

        # Plan the mission
        mission = self.plan(
            planning_request.request.current_pose,
            planning_request.request.goal_pose,
            planning_request.request.timeout,
        )

        if not mission:
            self.get_logger().warning(
                "Mission planning failed. No feasible path was found from the"
                "current pose to the goal pose."
            )
            planning_request.abort()
        else:
            planning_request.succeed()
            result.mission = mission

        return result

    @abstractmethod
    def plan(
        self, start_pose: PoseStamped, goal_pose: PoseStamped, timeout: float
    ) -> list[Waypoint]:
        """Plan a sequence of waypoints from the start pose to the goal pose.

        Args:
            start_pose (PoseStamped): The pose to start planning from.
            goal_pose (PoseStamped): The desired pose.
            timeout (float): The maximum amount of time allowed for planning.

        Raises:
            NotImplementedError: This method has not yet been implemented.

        Returns:
            list[Waypoint]: The resulting path found by the algorithm.
        """
        raise NotImplementedError("This method has not yet been implemented!")


class PrePlannedMissionPlanner(MissionPlanner):
    """Mission planner which loads pre-planned missions from a configuration file."""

    def __init__(self) -> None:
        """Create a new pre-planned mission planner.

        Raises:
            ValueError: The name of the mission to load was not defined.
        """
        super().__init__("preplanned_mission_planner")

        self.declare_parameters(
            "",
            [
                ("mission_name", ""),
                (
                    "mission_library_folder",
                    os.path.join(
                        get_package_share_directory("angler_planning"),
                        "missions",
                        "library",
                    ),
                ),
            ],
        )

        # Get the pre-planned mission
        mission_name = (
            self.get_parameter("mission_name").get_parameter_value().string_value
        )

        if mission_name == "":
            raise ValueError("Mission name not provided.")

        library_folder = (
            self.get_parameter("mission_library_folder")
            .get_parameter_value()
            .string_value
        )
        loader = PrePlannedMissionLoader(library_folder)

        self.mission = loader.load_mission(mission_name)

        # Notify users early that the system failed to load the mission
        if len(self.mission) <= 0:
            self.get_logger().warning(
                (
                    f"Failed to load the mission {mission_name} from {library_folder}"
                    f" or the desired mission has no waypoints. Please verify that the"
                    " provided mission name is accurate and that the library folder"
                    " is accessible to this node at runtime."
                )
            )

        self.get_logger().info(f"Successfully loaded mission '{mission_name}'.")

    def plan(
        self, start_pose: PoseStamped, goal_pose: PoseStamped, timeout: float
    ) -> list[Waypoint]:
        """Load a pre-planned mission.

        Args:
            start_pose (PoseStamped): The pose to start planning from.
            goal_pose (PoseStamped): The desired pose.
            timeout (float): The maximum amount of time allowed for planning.

        Returns:
            list[Waypoint]: The pre-planned mission.
        """
        return self.mission


def main_preplanned(args: list[str] | None = None):
    """Run the pre-planned waypoint planner."""
    rclpy.init(args=args)

    node = PrePlannedMissionPlanner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
