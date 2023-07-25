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

import numpy as np


class Constraint(ABC):
    """Base class for defining a constraint."""

    # The name of the constraint. This is used to reference the constraint at launch.
    name: str = ""

    def __init__(
        self,
        gain: float,
        priority: float,
    ) -> None:
        """Create a new constraint.

        Args:
            gain: The constraint gain.
            priority: The constraint priority in the constraint priority list.
        """
        self.gain = gain
        self.priority = priority

    @property
    def jacobian(self) -> np.ndarray:
        """Get the Jacobian for a constraint.

        Raises:
            NotImplementedError: This method has not yet been implemented.

        Returns:
            The task Jacobian.
        """
        raise NotImplementedError("This method has not yet been implemented!")

    @property
    def error(self) -> np.ndarray:
        """Get the reference signal for a constraint.

        Raises:
            NotImplementedError: This method has not yet been implemented.

        Returns:
            The constraint reference signal.
        """
        raise NotImplementedError("This method has not yet been implemented!")

    @abstractmethod
    def update(self, *args, **kwargs) -> None:
        """Update the constraint context.

        Raises:
            NotImplementedError: This method has not yet been implemented.
        """
        raise NotImplementedError("This method has not yet been implemented!")


class EqualityConstraint(Constraint):
    """Constraint which drives the system to a single value."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new equality constraint.

        Args:
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
        """
        super().__init__(gain, priority)


class SetConstraint(Constraint):
    """Constraint which limits a task value to a range."""

    def __init__(
        self,
        upper: float,
        lower: float,
        activation_threshold: float,
        deactivation_threshold: float,
        gain: float,
        priority: float,
    ) -> None:
        """Create a new set constraint.

        Args:
            name: The name of the constraint.
            upper: The constraint upper bound.
            lower: The constraint lower bound.
            activation_threshold: The threshold at which the task is activated.
            deactivation_threshold: The distance from the set boundaries at which the
                task becomes deactivated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
        """
        super().__init__(gain, priority)

        self.range = (lower, upper)
        self.activation_threshold = activation_threshold
        self.deactivation_threshold = deactivation_threshold
