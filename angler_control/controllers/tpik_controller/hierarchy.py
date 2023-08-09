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

import itertools
import os
from typing import Any

import numpy as np
import yaml  # type: ignore
from controllers.tpik_controller.constraints import (
    Constraint,
    EqualityConstraint,
    SetConstraint,
)
from controllers.tpik_controller.tasks import task_library


class TaskHierarchy:
    """Interface for loading and managing a task hierarchy."""

    def __init__(self, tasks: list[Constraint]) -> None:
        """Create a new task hierarchy."""
        # Make sure that the tasks are sorted according to their priority
        self.tasks = sorted(tasks, key=lambda task: task.priority)

    @property
    def active_task_hierarchy(self) -> list[Constraint]:
        """Get the set of activated tasks, ordered from highest priority to lowest.

        Returns:
            The active task hierarchy.
        """
        return [
            task
            for task in self.tasks
            if (isinstance(task, SetConstraint) and task.active)
            or isinstance(task, EqualityConstraint)
        ]

    @property
    def hierarchies(self) -> list[list[SetConstraint | EqualityConstraint]]:
        """Get the set of all potential mode combinations for the active task hierarchy.

        Returns:
            The active task hierarchy modes.
        """
        n_set_tasks = len(
            [
                set_task
                for set_task in self.active_task_hierarchy
                if isinstance(set_task, SetConstraint)
            ]
        )

        # Get all possible combinations of modes
        combinations = list(itertools.product([0, 1], repeat=n_set_tasks))

        # Sort the combinations according to their restrictiveness: those with the most
        # 1's will appear at lower indices
        combinations = sorted(combinations, key=lambda row: -np.sum(row))

        hierarchy_combinations: list[list[SetConstraint | EqualityConstraint]] = []

        # If there are no set-based tasks active, then just return the equality tasks
        if not combinations:
            hierarchy_combinations.append(
                [
                    eq_task
                    for eq_task in self.tasks
                    if isinstance(eq_task, EqualityConstraint)
                ]
            )
            return hierarchy_combinations

        # Generate a set of all potential hierarchy combinations using the mode map
        for combination in combinations:
            # Keep track of which set-based task we are referencing
            set_task_idx = 0

            hierarchy: list[SetConstraint | EqualityConstraint] = []

            for task in self.active_task_hierarchy:
                if isinstance(task, SetConstraint):
                    if combination[set_task_idx]:
                        hierarchy.append(task)
                    set_task_idx += 1
                elif isinstance(task, EqualityConstraint):
                    hierarchy.append(task)

            # hierarchy
            hierarchy_combinations.append(hierarchy)

        return hierarchy_combinations

    @staticmethod
    def load_tasks_from_path(filepath: str):
        """Load the desired tasks from a YAML configuration file.

        Args:
            filepath: The full path to the YAML configuration file.

        Raises:
            ValueError: Invalid file path provide.
            ValueError: Invalid task configuration provided.
        """
        if not os.path.isfile(filepath):
            raise ValueError(
                "The provided task configuration file is not valid. Please ensure that"
                " the path is defined correctly and is visible to ROS at runtime:"
                f" {filepath}"
            )

        with open(filepath, encoding="utf-8") as task_f:
            hierarchy: list[dict[str, Any]] = yaml.safe_load(task_f)

            tasks: list[Constraint] = []

            for task in hierarchy:
                name = task.pop("task")

                if name is None or name not in task_library.keys():
                    raise ValueError(
                        "The provided task hierarchy contains an invalid task"
                        f" type: {name}"
                    )

                constraint = task_library[name].create_task_from_params(  # type: ignore # noqa
                    **task,
                )

                tasks.append(constraint)

        return TaskHierarchy(tasks)
