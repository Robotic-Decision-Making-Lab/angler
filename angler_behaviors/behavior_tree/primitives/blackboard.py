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

from typing import Any, Callable

import py_trees


class FunctionOfBlackboardVariables(py_trees.behaviour.Behaviour):
    """Apply a function to a set of blackboard variables and save the result.

    This implementation is inspired by some work done by Charles Dawson on a project
    that we worked on together.
    """

    def __init__(
        self,
        name: str,
        input_keys: list[str],
        output_key: str,
        function: Callable[..., Any],
    ):
        """Create a new FunctionOfBlackboardVariables behavior.

        Args:
            name: The name of the behavior.
            input_keys: The blackboard keys whose values will be passed as arguments to
                the provided function.
            output_key: The blackboard key where the result of the function should
                be saved.
            function: A function to call using the values stored at the input keys as
                (positional) arguments.
        """
        super().__init__(name)

        self.input_keys = input_keys
        self.output_key = output_key
        self.function = function

        self.blackboard = self.attach_blackboard_client()

        for key in self.input_keys:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

        self.blackboard.register_key(
            key=self.output_key, access=py_trees.common.Access.WRITE
        )

    def initialise(self) -> None:
        """Initialize the behavior."""
        self.blackboard.unset(self.output_key)

    def update(self) -> py_trees.common.Status:
        """Apply the function to the saved blackboard variables.

        Returns:
            `SUCCESS` if the function is able to access all required keys and execute
            the provided function; `FAILURE`, otherwise.
        """
        for key in self.input_keys:
            if not self.blackboard.exists(key):
                self.logger.error(
                    f"Tried to read {key} from the blackboard, but it doesn't exist!"
                )

                return py_trees.common.Status.FAILURE

        args = [self.blackboard.get(key) for key in self.input_keys]
        result = None

        try:
            result = self.function(*args)
        except Exception as e:
            self.logger.error(
                f"Tried to apply function to {self.input_keys}, but failed: {e}"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.set(self.output_key, result)

        return py_trees.common.Status.SUCCESS
