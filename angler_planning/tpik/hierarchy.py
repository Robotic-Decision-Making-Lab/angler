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
from tpik.tasks import (
    EndEffectorPoseTask,
    JointLimitTask,
    ManipulatorConfigurationTask,
    VehicleOrientationTask,
    VehicleRollPitchTask,
    VehicleYawTask,
)
from tpik.tasks.constraint import EqualityConstraint, SetConstraint

_task_library = {
    "joint_limit_set": JointLimitTask,
    "end_effector_pose_eq": EndEffectorPoseTask,
    "vehicle_roll_pitch_eq": VehicleRollPitchTask,
    "vehicle_yaw_eq": VehicleYawTask,
    "vehicle_orientation_eq": VehicleOrientationTask,
    "joint_configuration_eq": ManipulatorConfigurationTask,
}


def task_factory(name: str, **kwargs) -> EqualityConstraint | SetConstraint:
    return _task_library[name](**kwargs)


class TaskHierarchy:
    _hierarchy: list[SetConstraint | EqualityConstraint] = []

    @classmethod
    def load_hierarchy_from_path(cls, filepath: str):
        if not os.path.isfile(filepath):
            raise ValueError(
                "The provided task configuration file is not valid. Please ensure that"
                " the path is defined correctly and is visible to ROS at runtime."
            )

        with open(filepath, encoding="utf-8") as task_f:
            hierarchy: list[dict[str, Any]] = yaml.safe_load(task_f)

            for task in hierarchy:
                name = task.pop("task")

                if name is None:
                    raise ValueError(
                        "The provided task hierarchy contains an invalid task"
                        f" name: {name}"
                    )

                bleh = task_factory(name, **task)
