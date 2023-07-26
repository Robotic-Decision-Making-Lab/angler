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

from abc import ABC, abstractmethod
from collections import namedtuple
from typing import Any

import numpy as np

Threshold = namedtuple("Threshold", ["lower", "upper"])


class Task(ABC):
    """Base class for defining a task."""

    # The name of the task. This is used to reference the task at launch.
    name: str = ""

    def __init__(
        self,
        gain: float,
        priority: float,
    ) -> None:
        """Create a new task.

        Args:
            gain: The task gain.
            priority: The task priority in the task priority list.
        """
        self.gain = gain
        self.priority = priority
        self.desired_value: Any | None = None  # The desired task value
        self.current_value: Any | None = None  # The current task value
        self.desired_value_dot: Any | None = None  # The desired task dynamics

    @property
    def jacobian(self) -> np.ndarray:
        """Get the Jacobian for a task.

        Raises:
            NotImplementedError: This method has not yet been implemented.

        Returns:
            The task Jacobian.
        """
        raise NotImplementedError("This method has not yet been implemented!")

    @property
    def error(self) -> np.ndarray:
        """Get the reference signal for a task.

        Raises:
            NotImplementedError: This method has not yet been implemented.

        Returns:
            The task reference signal.
        """
        raise NotImplementedError("This method has not yet been implemented!")

    @abstractmethod
    def update(self, *args, **kwargs) -> None:
        """Update the task context.

        Raises:
            NotImplementedError: This method has not yet been implemented.
        """
        raise NotImplementedError("This method has not yet been implemented!")


class EqualityTask(Task):
    """Constraint which drives the system to a single value."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new equality task.

        Args:
            gain: The task gain to use for closed-loop control.
            priority: The task priority.
        """
        super().__init__(gain, priority)


class SetTask(Task):
    """Constraint which limits a task value to a range."""

    def __init__(
        self,
        physical_upper: float,
        physical_lower: float,
        safety_upper: float,
        safety_lower: float,
        activation_threshold: float,
        gain: float,
        priority: float,
    ) -> None:
        """Create a new set task.

        Args:
            name: The name of the task.
            physical_upper: The task physical upper bound.
            physical_lower: The task physical lower bound.
            safety_upper: The upper safety limit used to create a buffer from the
                upper physical limit.
            safety_lower: The lower safety limit used to create a buffer from the lower
                physical limit.
            activation_threshold: The distance from safety thresholds at which the task
                should become activated.
            gain: The task gain to use for closed-loop control.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.physical_threshold = Threshold(physical_lower, physical_upper)
        self.safety_threshold = Threshold(safety_lower, safety_upper)
        self.activation_threshold = Threshold(
            safety_lower + activation_threshold, safety_upper - activation_threshold
        )
        self.active = False

    def set_task_active(self, value: float) -> bool:
        """Set the task activity.

        Note that this also sets the desired task value according to the provided
        thresholds.

        Args:
            value: The current value of the variable which the task applies to.

        Returns:
            True if the task was activated; false, otherwise.
        """
        active = False

        if value < self.activation_threshold.lower:
            self.desired_value = self.safety_threshold.lower
            active = True
        elif value > self.activation_threshold.upper:
            self.desired_value = self.safety_threshold.upper
            active = True

        self.active = active

        return self.active
