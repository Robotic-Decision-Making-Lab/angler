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
from typing import Any

import yaml  # type: ignore
from tpik.constraint import Constraint
from tpik.tasks import (
    EndEffectorPose,
    JointLimit,
    ManipulatorConfiguration,
    VehicleOrientation,
    VehicleRollPitch,
    VehicleYaw,
)

_task_library = {
    task.name: task  # type: ignore
    for task in [
        EndEffectorPose,
        JointLimit,
        ManipulatorConfiguration,
        VehicleOrientation,
        VehicleRollPitch,
        VehicleYaw,
    ]
}


class TaskHierarchy:
    """Interface for loading and managing a task hierarchy."""

    def __init__(self, constraints: list[Constraint]) -> None:
        """Create a new task hierarchy."""
        self.constraints = constraints
        self.activated_task_hierarchy: list[Constraint] = []

    def update_task_hierarchy(self):
        ...

    @staticmethod
    def load_constraints_from_path(filepath: str):
        """Load the desired constraints from a YAML configuration file.

        Args:
            filepath: The full path to the YAML configuration file.

        Raises:
            ValueError: Invalid file path provide.
            ValueError: Invalid task configuration provided.
        """
        if not os.path.isfile(filepath):
            raise ValueError(
                "The provided task configuration file is not valid. Please ensure that"
                " the path is defined correctly and is visible to ROS at runtime."
            )

        with open(filepath, encoding="utf-8") as task_f:
            hierarchy: list[dict[str, Any]] = yaml.safe_load(task_f)

            constraints: list[Constraint] = []

            for constraint in hierarchy:
                name = constraint.pop("task")

                if name is None or name not in _task_library.keys():
                    raise ValueError(
                        "The provided task hierarchy contains an invalid task"
                        f" type: {name}"
                    )

                task = _task_library[name].create_task_from_params(  # type: ignore
                    **constraint,
                )

                constraints.append(task)

        return TaskHierarchy(constraints)
