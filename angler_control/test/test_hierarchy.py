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

import pytest
from ament_index_python import get_package_share_directory
from tpik.constraint import EqualityTask, SetTask
from tpik.hierarchy import TaskHierarchy


@pytest.fixture
def hierarchy():
    """Load a task hierarchy from a configuration file for testing.

    Yields:
        A task hierarchy with tasks defined according to the config file.
    """
    task_path = os.path.join(
        get_package_share_directory("angler_planning"),
        "test",
        "resources",
        "test_tasks.yaml",
    )

    yield TaskHierarchy.load_tasks_from_path(task_path)


def test_active_task_hierarchy(hierarchy: TaskHierarchy) -> None:
    """Test that the active task hierarchy gets all active tasks.

    Args:
        hierarchy: The task hierarchy.
    """
    # There should only be one equality task at the moment
    assert len(hierarchy.active_task_hierarchy) == 1 and isinstance(
        hierarchy.active_task_hierarchy[0], EqualityTask
    )

    # Now activate the set task
    for task in hierarchy.tasks:
        if isinstance(task, SetTask):
            task.active = True

    assert len(hierarchy.active_task_hierarchy) == 2


def test_modes(hierarchy: TaskHierarchy) -> None:
    """Test that the combinations of hierarchies is properly generated.

    Args:
        hierarchy: The task hierarchy.
    """
    assert len(hierarchy.hierarchies) == 1

    # Now activate the set task
    for task in hierarchy.tasks:
        if isinstance(task, SetTask):
            task.active = True

    assert len(hierarchy.hierarchies) == 2  # 2^1 = 2
