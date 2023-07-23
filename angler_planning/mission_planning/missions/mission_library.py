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

import json
import os

from geometry_msgs.msg import Transform, Twist
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class Mission:
    """A sequence of waypoints for the UVMS to track."""

    def __init__(
        self, name: str, frame: str, waypoints: list[dict[str, dict[str, float]]]
    ) -> None:
        """Create a new mission.

        Args:
            name: The unique mission name.
            frame: The coordinate frame that the mission is defined in.
            waypoints: The sequence of waypoints that the UVMS should track.
        """
        if len(waypoints) <= 0:
            raise ValueError(
                "All missions must define at least one waypoint. However, mission"
                f" '{name}' does not define any waypoints."
            )

        self.name = name

        # Construct the joint trajectory
        self.waypoints = MultiDOFJointTrajectory()
        self.waypoints.header.frame_id = frame
        self.waypoints.joint_names = ["end_effector"]
        self.waypoints.points = [
            self._create_waypoint_msg(waypoint) for waypoint in waypoints
        ]

    @staticmethod
    def _create_waypoint_msg(
        waypoint: dict[str, dict[str, float]]
    ) -> MultiDOFJointTrajectoryPoint:
        """Create a new Waypoint message from the JSON data.

        Args:
            waypoint: The waypoint definition.

        Returns:
            A MultiDOFJointTrajectoryPoint message.
        """
        point = MultiDOFJointTrajectoryPoint()

        # Specify the end effector pose
        transform = Transform()

        transform.translation.x = waypoint["position"]["x"]
        transform.translation.y = waypoint["position"]["y"]
        transform.translation.z = waypoint["position"]["z"]

        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ) = R.from_euler(
            "xyz",
            [
                waypoint["position"]["roll"],
                waypoint["position"]["pitch"],
                waypoint["position"]["yaw"],
            ],
        ).as_quat(
            False
        )

        point.transforms.append(transform)  # type: ignore

        # Specify the end effector velocity
        velocity = Twist()

        velocity.linear.x = waypoint["velocity"]["x"]
        velocity.linear.y = waypoint["velocity"]["y"]
        velocity.linear.z = waypoint["velocity"]["z"]

        velocity.angular.x = waypoint["velocity"]["rx"]
        velocity.angular.y = waypoint["velocity"]["ry"]
        velocity.angular.z = waypoint["velocity"]["rz"]

        point.velocities.append(velocity)  # type: ignore

        # Specify the end effector acceleration
        acceleration = Twist()

        acceleration.linear.x = waypoint["acceleration"]["x"]
        acceleration.linear.y = waypoint["acceleration"]["y"]
        acceleration.linear.z = waypoint["acceleration"]["z"]

        acceleration.angular.x = waypoint["acceleration"]["rx"]
        acceleration.angular.y = waypoint["acceleration"]["ry"]
        acceleration.angular.z = waypoint["acceleration"]["rz"]

        point.accelerations.append(acceleration)  # type: ignore

        return point


class MissionLibrary:
    """A library of pre-planned missions."""

    _library: dict[str, Mission] = {}

    @classmethod
    def add_mission(cls, mission: Mission) -> None:
        """Add a mission to the library.

        Args:
            mission: The mission to save to the library.

        Raises:
            ValueError: A mission with this name already exists in the library.
        """
        if mission.name in cls._library:
            raise ValueError(
                f"A mission with the name '{mission.name}' already exists in the"
                f" library."
            )

        cls._library[mission.name] = mission

    @classmethod
    def load_mission_library_from_path(cls, library_path: str) -> None:
        """Load a collection of pre-defined missions from a directory.

        All missions must be defined as JSON files.

        Args:
            library_path: The full path to the directory to load.

        Raises:
            ValueError: The provided library path is invalid/not accessible.
            ValueError: Multiple missions share a name. Missions should have unique
                names.
        """
        if not os.path.isdir(library_path):
            raise ValueError(
                "The library path provided is not a valid path. Please make sure that"
                " the path is defined correctly and is visible to ROS at runtime"
            )

        # Load all mission files in the directory
        mission_files = [
            os.path.join(library_path, f)
            for f in os.listdir(library_path)
            if f.endswith(".json") and os.path.isfile(os.path.join(library_path, f))
        ]

        # Create a new mission and save it to the library
        for f in mission_files:
            with open(f, encoding="utf-8") as mission_f:
                mission = Mission(**json.load(mission_f))
                cls.add_mission(mission)

    @classmethod
    def select_mission(cls, name: str) -> Mission:
        """Retrieve a mission from the library.

        Args:
            name: The name of the mission to retrieve.

        Returns:
            The mission with the provided name.
        """
        return cls._library[name]
