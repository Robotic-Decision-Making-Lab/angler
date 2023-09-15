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

import copy
from typing import Any, Callable

import py_trees
from py_trees_ros.subscribers import Handler
from rclpy.qos import QoSProfile


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


class ToBlackboardNonBlocking(Handler):
    """Write a ROS message to the blackboard.

    This class is a port of the `py_trees_ros.subscribers.ToBlackboard`, but modifies
    it to make the behavior non-blocking.
    """

    def __init__(
        self,
        name: str,
        topic_name: str,
        topic_type: Any,
        qos_profile: QoSProfile,
        blackboard_variables: dict[str, Any],
        initialise_variables: dict[str, Any] | None = None,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
    ):
        """Create a new non-blocking `ToBlackboard` behavior.

        Args:
            name: The name of the behavior.
            topic_name: The name of the topic to connect to.
            topic_type: The class of the message type (e.g., :obj:`std_msgs.msg.String`)
            qos_profile: The QoSProfile for the subscriber.
            blackboard_variables: The blackboard variable to write to. This should be
                formatted as: {names (keys): message subfields (values)}. Use a value of
                `None` to indicate that the entire message should be saved.
            initialise_variables: Initialize the blackboard variables to some defaults.
            clearing_policy: When to clear the data. Defaults to
                `py_trees.common.ClearingPolicy.ON_INITIALISE`.
        """
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=topic_type,
            qos_profile=qos_profile,
            clearing_policy=clearing_policy,
        )
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.logger = py_trees.logging.Logger(f"{self.name}")
        self.blackboard_variable_mapping = blackboard_variables

        # Register the keys
        for name in self.blackboard_variable_mapping:
            self.blackboard.register_key(key=name, access=py_trees.common.Access.WRITE)

        # Set the keys to some initial values
        if initialise_variables is not None:
            for k, v in initialise_variables.items():
                self.blackboard.set(k, v)

    def update(self):
        """Write the data (if available) to the blackboard.

        Returns:
            This behavior always returns `SUCCESS`.
        """
        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return py_trees.common.Status.SUCCESS
            else:
                for k, v in self.blackboard_variable_mapping.items():
                    if v is None:
                        self.blackboard.set(k, self.msg, overwrite=True)
                    else:
                        fields = v.split(".")
                        value = copy.copy(self.msg)
                        for field in fields:
                            value = getattr(value, field)
                            self.blackboard.set(k, value, overwrite=True)

                self.feedback_message = "saved incoming message"

                if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
                    self.msg = None

                return py_trees.common.Status.SUCCESS
