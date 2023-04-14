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

from scipy.spatial.transform import Rotation as R

from angler_msgs.msg import Waypoint


class Mission:
    """A sequence of waypoints for the UVMS to track."""

    def __init__(
        self, name: str, frame: str, waypoints: list[dict[str, float]]
    ) -> None:
        """Create a new mission.

        Example:
            >>> mission = Mission(
            ...     name="my_awesome_mission",
            ...     frame="map",
            ...     waypoints=[
            ...         {
            ...             "x": 0.0,
            ...             "y": 0.0,
            ...             "z": 0.0,
            ...             "roll": 0.0,
            ...             "pitch": 0.0,
            ...             "yaw": 0.0,
            ...             "acceleration": 0.0,
            ...             "velocity": 1.0,
            ...         },
            ...     ],
            ... )

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
        self.waypoints = [
            self._create_waypoint_msg(frame, waypoint) for waypoint in waypoints
        ]

    @staticmethod
    def _create_waypoint_msg(frame: str, waypoint: dict[str, float]) -> Waypoint:
        """Create a new Waypoint message from the JSON data.

        Args:
            frame: The coordinate frame that the waypoint is defined in.
            waypoint: The waypoint definition.

        Returns:
            A Waypoint ROS message.
        """
        msg = Waypoint()

        msg.pose.header.frame_id = frame

        msg.pose.pose.position.x = waypoint["x"]
        msg.pose.pose.position.y = waypoint["y"]
        msg.pose.pose.position.z = waypoint["z"]

        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = R.from_euler(
            "xyz", (waypoint["roll"], waypoint["pitch"], waypoint["yaw"])
        ).as_quat()

        msg.acceleration = waypoint["acceleration"]
        msg.velocity = waypoint["velocity"]

        return msg


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
