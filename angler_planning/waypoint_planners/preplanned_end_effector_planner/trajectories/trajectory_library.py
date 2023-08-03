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
from moveit_msgs.msg import RobotTrajectory
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


def create_waypoint_msg(
    waypoint: dict[str, dict[str, float]]
) -> MultiDOFJointTrajectoryPoint:
    """Create a new Waypoint message from the JSON data.

    Args:
        waypoint: The waypoint definition.

    Returns:
        A MultiDOFJointTrajectoryPoint message.
    """
    point = MultiDOFJointTrajectoryPoint()

    # Get the desired pose
    tf = Transform()

    tf.translation.x, tf.translation.y, tf.translation.z, rx, ry, rz = waypoint[
        "transform"
    ].values()

    tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w = R.from_euler(
        "xyz", [rx, ry, rz]
    ).as_quat()

    # Get the desired velocity
    vel = Twist()

    (
        vel.linear.x,
        vel.linear.y,
        vel.linear.z,
        vel.angular.x,
        vel.angular.y,
        vel.angular.z,
    ) = waypoint["velocity"].values()

    # Finally, get the acceleration
    accel = Twist()

    (
        accel.linear.x,
        accel.linear.y,
        accel.linear.z,
        accel.angular.x,
        accel.angular.y,
        accel.angular.z,
    ) = waypoint["acceleration"].values()

    point.transforms = [point]
    point.velocities = [vel]
    point.accelerations = [accel]

    return point


def create_robot_trajectory_msg(
    parent: str, child: str, waypoints: list[dict[str, dict[str, float]]]
) -> RobotTrajectory:
    """Creata robot trajectory from JSON parameters.

    Args:
        frame: The source frame that the waypoints are defined in.
        child: The child frame that the waypoints describe the transform to.
        waypoints: The collection of waypoints to define.

    Returns:
        A RobotTrajectory message.
    """
    traj = MultiDOFJointTrajectory()

    traj.header.frame_id = parent
    traj.joint_names = [child]
    traj.points = [create_waypoint_msg(wp) for wp in waypoints]

    msg = RobotTrajectory()
    msg.multi_dof_joint_trajectory = traj

    return msg


class TrajectoryLibrary:
    """A library of pre-planned end-effector trajectories."""

    _library: dict[str, RobotTrajectory] = {}

    @classmethod
    def add_trajectory(cls, name: str, trajectory: RobotTrajectory) -> None:
        """Add a new trajectory to the trajectory library.

        Args:
            name: The trajectory name.
            trajectory: The RobotTrajectory message.

        Raises:
            ValueError: Duplicate trajectory names.
        """
        if name not in cls._library:
            cls._library[name] = trajectory
        else:
            raise ValueError(
                "Duplicate trajectory names provided in the trajectory library!"
            )

    @classmethod
    def load_library_from_path(cls, library_path: str) -> None:
        """Load the trajectory library from a path.

        Args:
            library_path: The full path to the trajectory library.

        Raises:
            ValueError: Invalid filepath provided.
        """
        if not os.path.isdir(library_path):
            raise ValueError(
                "The library path provided is not a valid path. Please make sure that"
                " the path is defined correctly and is visible to ROS at runtime"
            )

        # Load all trajectory files in the directory
        mission_files = [
            os.path.join(library_path, f)
            for f in os.listdir(library_path)
            if f.endswith(".json") and os.path.isfile(os.path.join(library_path, f))
        ]

        # Create a new trajectory and save it to the library
        for f in mission_files:
            with open(f, encoding="utf-8") as trajectory_f:
                params = json.load(trajectory_f)
                name = params.pop("name")
                trajectory = create_robot_trajectory_msg(**params)
                cls.add_trajectory(name, trajectory)

    @classmethod
    def select_trajectory(cls, name: str) -> RobotTrajectory:
        """Retrieve a trajectory from the library.

        Args:
            name: The name of the trajectory to retrieve.

        Returns:
            The trajectory with the provided name.
        """
        return cls._library[name]
