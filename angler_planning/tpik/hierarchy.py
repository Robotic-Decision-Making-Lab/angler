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

import os
from collections import namedtuple
from typing import Any

import tpik.jacobian as J
import yaml  # type: ignore
from tpik.constraint import EqualityConstraint, SetConstraint

TaskWrapper = namedtuple("TaskWrapper", ["name", "task", "jacobian_func"])


_task_library = {
    "joint_limit_set": (
        SetConstraint,
        J.calculate_joint_limit_jacobian,
    ),
    "vehicle_orientation_eq": (
        EqualityConstraint,
        J.calculate_vehicle_orientation_jacobian,
    ),
    "vehicle_roll_pitch_eq": (
        EqualityConstraint,
        J.calculate_vehicle_roll_pitch_jacobian,
    ),
    "vehicle_yaw_eq": (
        EqualityConstraint,
        J.calculate_vehicle_yaw_jacobian,
    ),
    "end_effector_pose_eq": (
        EqualityConstraint,
        J.calculate_uvms_jacobian,
    ),
    "joint_config_eq": (
        EqualityConstraint,
        J.calculate_joint_configuration_jacobian,
    ),
    "collision_plane_set": (
        EqualityConstraint,
        J.calculate_vehicle_manipulator_collision_avoidance_jacobian,
    ),
}


class TaskHierarchy:
    def __init__(self) -> None:
        self._constraints: list[TaskWrapper] = []
        self.hierarchy: list[TaskWrapper] = []

    def load_constraints_from_path(self, filepath: str):
        if not os.path.isfile(filepath):
            raise ValueError(
                "The provided task configuration file is not valid. Please ensure that"
                " the path is defined correctly and is visible to ROS at runtime."
            )

        with open(filepath, encoding="utf-8") as task_f:
            hierarchy: list[dict[str, Any]] = yaml.safe_load(task_f)

            for task in hierarchy:
                name = task.pop("task")

                if name is None or name not in _task_library.keys():
                    raise ValueError(
                        "The provided task hierarchy contains an invalid task"
                        f" name: {name}"
                    )

                wrapper = TaskWrapper(
                    name, _task_library[name][0](**task), _task_library[name][1]
                )

                self._constraints.append(wrapper)
