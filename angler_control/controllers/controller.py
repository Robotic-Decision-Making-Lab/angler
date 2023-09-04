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

from moveit_msgs.msg import RobotState
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool


class BaseController(ABC, Node):
    """Base class for a whole-body controller."""

    def __init__(self, controller_name: str) -> None:
        """Create a new controller interface.

        Args:
            controller_name: The name of the controller.
        """
        ABC.__init__(self)
        Node.__init__(self, controller_name)

        self.declare_parameter("control_rate", 30.0)

        self.armed = False
        self.robot_state = RobotState()
        self.dt = 1 / (
            self.get_parameter("control_rate").get_parameter_value().double_value
        )

        # Services
        self.arm_controller_srv = self.create_service(
            SetBool, "/angler/cmd/arm", self.arm_controller_cb, callback_group=ReentrantCallbackGroup()
        )

        # Subscribers
        self.robot_state_sub = self.create_subscription(
            RobotState,
            "/angler/state",
            self._robot_state_cb,
            qos_profile=qos_profile_sensor_data,
        )

        self.control_loop_timer = self.create_timer(self.dt, self._update, MutuallyExclusiveCallbackGroup())

    def on_robot_state_update(self, state: RobotState) -> None:
        """Execute this function on robot state update.

        Args:
            state: The most recent robot state update.
        """
        # Don't do anything with the state by default
        ...

    def _robot_state_cb(self, state: RobotState) -> None:
        """Update the current robot state and run the `on_robot_state` callback.

        Args:
            state: The current robot state.
        """
        self.on_robot_state_update(state)
        self.state = state

    @abstractmethod
    def on_update(self) -> None:
        """Execute this function on control loop iteration."""
        raise NotImplementedError("This method has not yet been implemented!")

    def _update(self) -> None:
        """Run the control loop."""
        if not self.armed:
            # Don't do anything if we aren't armed
            ...
        else:
            self.on_update()

    def on_arm(self) -> bool:
        """Execute this function on system arming.

        Returns:
            Whether or not the arming procedure succeeded.
        """
        # Default to indicating success
        return True

    def on_disarm(self) -> bool:
        """Execute this function on system disarming.

        Returns:
            Whether or not the disarming procedure succeeded.
        """
        # Default to indicating success
        return True

    def arm_controller_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Arm/disarm the controller.

        This can be overridden to support more complex pre-arming checks.

        Args:
            request: The arm/disarm request.
            response: The request result.

        Returns:
            The request result.
        """
        if request.data:
            result = self.on_arm()
            self.armed = self.armed if not result else result
        else:
            result = self.on_disarm()
            self.armed = self.armed if not result else not result

        if not result:
            response.success = False
            response.message = (
                "Failed to "
                + ("arm" if request.data else "disarm")
                + " the Angler controller!"
            )
            self.get_logger().warning(response.message)
        else:
            response.success = True
            response.message = (
                "Successfully "
                + ("armed" if self.armed else "disarmed")
                + " the Angler controller!"
            )
            self.get_logger().info(response.message)

        return response
