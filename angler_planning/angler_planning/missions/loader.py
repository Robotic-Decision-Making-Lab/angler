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

import yaml  # type: ignore
from scipy.spatial.transform import Rotation as R

from angler_msgs.msg import Waypoint


class PrePlannedMissionLoader:
    """Interface for loading pre-planned missions."""

    def __init__(self, library_folder: str) -> None:
        """Create a new pre-planned mission loader.

        Args:
            library_folder (str): The full path to the folder that is holding the
                pre-planned missions.
        """
        # Load all pre-planned mission files
        self.mission_files = [
            os.path.join(library_folder, f)
            for f in os.listdir(library_folder)
            if os.path.isfile(os.path.join(library_folder, f))
        ]

    def find_mission_file(self, mission_name: str) -> str | None:
        """Find the mission file to use given the mission name.

        NOTE: If multiple missions have the same name, this will return the first one
        encountered.

        Args:
            mission_name (str): The name of the mission to load.

        Returns:
            str | None: The full path to the mission file with the given name. If no
                file is found, returns None.
        """
        for filename in self.mission_files:
            with open(filename, "r", encoding="utf-8") as mission_file:
                data = yaml.safe_load(mission_file)

                if data["name"] == mission_name:
                    return filename

        return None

    @staticmethod
    def get_waypoint_msg(
        position: tuple[int, int, int],
        orientation: tuple[int, int, int],
        acceleration: float,
        velocity: float,
        frame: str = "map",
    ) -> Waypoint:
        """Create a Waypoint message using the provided information.

        Args:
            position (tuple[int, int, int]): The position of the waypoint (x, y, z).
            orientation (tuple[int, int, int]): The orientation of the waypoint
                (roll, pitch, yaw) in radians.
            acceleration (float): The desired acceleration when passing through
                the waypoint.
            velocity (float): The desired velocity when passing through the waypoint.
            frame (str): The coordinate frame that the pose is defined in.

        Returns:
            Waypoint: The resulting Waypoint message.
        """
        msg = Waypoint()

        msg.pose.header.frame_id = frame

        (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ) = position

        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = R.from_euler("xyz", orientation).as_quat()

        msg.acceleration = acceleration
        msg.velocity = velocity

        return msg

    @classmethod
    def get_mission_data(cls, filename: str) -> list[Waypoint]:
        """Convert the mission specification to a list of Waypoint messages.

        Args:
            filename (str): The name of the file whose data should be loaded.

        Returns:
            list[Waypoint]: The mission data converted to a list of Waypoint messages.
        """
        trajectories = []

        with open(filename, "r", encoding="utf-8") as mission_file:
            data = yaml.safe_load(mission_file)

            trajectories.extend(
                [
                    cls.get_waypoint_msg(
                        (
                            data["waypoints"][i]["x"],
                            data["waypoints"][i]["y"],
                            data["waypoints"][i]["z"],
                        ),
                        (
                            data["waypoints"][i]["roll"],
                            data["waypoints"][i]["pitch"],
                            data["waypoints"][i]["yaw"],
                        ),
                        data["waypoints"][i]["acceleration"],
                        data["waypoints"][i]["velocity"],
                    )
                    for i in range(len(data["waypoints"]))
                ]
            )

        return trajectories

    def load_mission(self, mission_name: str) -> list[Waypoint]:
        """Get the desired mission.

        Args:
            mission_name (str): _description_

        Returns:
            list[Waypoint]: _description_
        """
        mission_file = self.find_mission_file(mission_name)

        if mission_file is None:
            return []

        return self.get_mission_data(mission_file)
