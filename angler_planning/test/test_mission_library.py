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

import pytest
from mission_planning.missions.mission_library import Mission
from mission_planning.missions.mission_library import MissionLibrary as ml
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


@pytest.fixture(autouse=True)
def reset_library(monkeypatch) -> None:
    """Reset the mission library after each test."""
    monkeypatch.setattr(ml, "_library", {})


def test_create_mission() -> None:
    """Test that missions are properly created."""
    mission = Mission(
        "my_awesome_mission",
        "map",
        waypoints=[
            {
                "transform": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "velocity": {
                    "x": 0.5,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
            }
        ],
    )

    # Make sure that only one waypoint was created and added
    assert len(mission.waypoints.points) == 1

    # Check the coordinate frame
    assert mission.waypoints.header.frame_id == "map"

    # Select the first waypoint and check to make sure that the Waypoint message was
    # populated correctly
    waypoint: MultiDOFJointTrajectoryPoint = mission.waypoints.points[0]  # type: ignore

    # Check the position
    assert waypoint.transforms[0].translation.x == 1.0  # type: ignore
    assert waypoint.transforms[0].translation.y == 2.0  # type: ignore
    assert waypoint.transforms[0].translation.z == 3.0  # type: ignore

    assert waypoint.transforms[0].rotation.x == 0.0  # type: ignore
    assert waypoint.transforms[0].rotation.y == 0.0  # type: ignore
    assert waypoint.transforms[0].rotation.z == 0.0  # type: ignore
    assert waypoint.transforms[0].rotation.w == 1.0  # type: ignore


def test_create_mission_with_no_waypoints() -> None:
    """Test that mission creation fails when no waypoints are provided."""
    with pytest.raises(ValueError):
        Mission("my_awesome_mission", "map", waypoints=[])


def test_add_mission() -> None:
    """Test that missions are correctly added to the library."""
    mission = Mission(
        "my_awesome_mission",
        "map",
        waypoints=[
            {
                "transform": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "velocity": {
                    "x": 0.5,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
            }
        ],
    )

    # Test adding the mission
    ml.add_mission(mission)

    assert mission.name in ml._library
    assert mission == ml._library[mission.name]


def test_select_mission() -> None:
    """Test that missions are correctly retrieved when queried."""
    mission = Mission(
        "my_awesome_mission",
        "map",
        waypoints=[
            {
                "transform": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "velocity": {
                    "x": 0.5,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
            }
        ],
    )

    # Save the mission to the library
    ml.add_mission(mission)

    # Verify that the mission retrieved is the same as the one created
    assert mission == ml.select_mission(mission.name)


def test_duplicate_missions() -> None:
    """Test that adding duplicate missions to the library fails."""
    mission = Mission(
        "my_awesome_mission",
        "map",
        waypoints=[
            {
                "transform": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "velocity": {
                    "x": 0.5,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
                "acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                },
            }
        ],
    )

    # Adding duplicate missions should fail
    with pytest.raises(ValueError):
        ml.add_mission(mission)
        ml.add_mission(mission)
