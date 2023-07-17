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
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "acceleration": 0.0,
                "velocity": 1.0,
            },
        ],
    )

    # Make sure that only one waypoint was created and added
    assert len(mission.waypoints) == 1

    # Select the first waypoint and check to make sure that the Waypoint message was
    # populated correctly
    waypoint = mission.waypoints[0]

    # Check the coordinate frame
    assert waypoint.pose.header.frame_id == "map"

    # Check the position
    assert waypoint.pose.pose.position.x == 0.0
    assert waypoint.pose.pose.position.y == 0.0
    assert waypoint.pose.pose.position.z == 0.0

    # Check the orientation (which is converted to a quaternion)
    assert waypoint.pose.pose.orientation.x == 0.0
    assert waypoint.pose.pose.orientation.y == 0.0
    assert waypoint.pose.pose.orientation.z == 0.0
    assert waypoint.pose.pose.orientation.w == 1.0

    # Check the position and acceleration
    assert waypoint.acceleration == 0.0
    assert waypoint.velocity == 1.0


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
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "acceleration": 0.0,
                "velocity": 1.0,
            },
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
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "acceleration": 0.0,
                "velocity": 1.0,
            },
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
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "acceleration": 0.0,
                "velocity": 1.0,
            },
        ],
    )

    # Adding duplicate missions should fail
    with pytest.raises(ValueError):
        ml.add_mission(mission)
        ml.add_mission(mission)
