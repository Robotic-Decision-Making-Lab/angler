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

import controllers.robot_trajectory_controller.utils as controller_utils
import numpy as np
from geometry_msgs.msg import Transform, Twist
from rclpy.time import Duration, Time
from scipy.interpolate import CubicHermiteSpline, interp1d
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class MultiDOFTrajectory:
    """Wraps a MultiDOFJointTrajectory for sampling purposes.

    The MultiDOFTrajectory wrapper provides and interface for sampling points between
    a discrete set of MultiDOFJointTrajectoryPoint messages. This helps provide
    continuity between points.
    """

    def __init__(
        self,
        trajectory: MultiDOFJointTrajectory,
        current_state: MultiDOFJointTrajectoryPoint,
        current_time: Time,
    ) -> None:
        """Create a new MultiDOFJointTrajectory interface.

        Args:
            trajectory: The trajectory to create an interface for.
            current_state: The current state of the robot prior to beginning the
                trajectory.
            current_time: The current ROS timestamp.
        """
        self.trajectory = trajectory
        self.starting_state = current_state
        self.starting_time = current_time

    def sample(self, t: Time) -> MultiDOFJointTrajectoryPoint | None:
        """Sample the trajectory at a given timestamp.

        Args:
            t: The timestamp to sample the trajectory at.

        Returns:
            The point at the given timestamp if it exists; None, otherwise.
        """
        if len(self.trajectory.points) <= 0:
            return None

        # Attempting to sample before the initial state
        if t < self.starting_time:
            return None

        first_point = self.trajectory.points[0]  # type: ignore
        first_point_timestamp = controller_utils.add_ros_time_duration_msg(
            self.starting_time, first_point.time_from_start
        )

        if t < first_point_timestamp:
            return self.interpolate(
                self.starting_state,
                first_point,
                self.starting_time,
                first_point_timestamp,
                t,
            )

        for i in range(len(self.trajectory.points) - 1):
            point = self.trajectory.points[i]  # type: ignore
            next_point = self.trajectory.points[i + 1]  # type: ignore

            t0 = controller_utils.add_ros_time_duration_msg(
                self.starting_time, point.time_from_start
            )
            t1 = controller_utils.add_ros_time_duration_msg(
                self.starting_time, next_point.time_from_start
            )

            if t0 <= t < t1:
                return self.interpolate(point, next_point, t0, t1, t)

        # We are sampling past the trajectory now; return the last point in the
        # trajectory
        return self.trajectory.points[-1]  # type: ignore

    def interpolate(
        self,
        start_point: MultiDOFJointTrajectoryPoint,
        end_point: MultiDOFJointTrajectoryPoint,
        start_time: Time,
        end_time: Time,
        sample_time: Time,
    ) -> MultiDOFJointTrajectoryPoint:
        """Interpolate between two points and sample the resulting function.

        This method currently only supports interpolation between positions and
        velocities. Interpolation between points with accelerations is not yet
        supported.

        For position-only interfaces, linear interpolation is used. For position +
        velocity interfaces, cubic Hermite spline interpolation is used.

        Args:
            start_point: The segment starting point.
            end_point: The segment end point.
            start_time: The timestamp of the first point in the segment.
            end_time: The timestamp of the second point in the segment.
            sample_time: The timestamp at which to sample.

        Returns:
            The identified point at the provided sample time.
        """
        duration_from_start: Duration = sample_time - start_time
        duration_between_points: Duration = end_time - start_time

        has_velocity = start_point.velocities and end_point.velocities

        # Check if the sample time is before the start point or if the sample time is
        # after the end point
        if (
            duration_from_start.nanoseconds < 0
            or duration_from_start.nanoseconds > duration_between_points.nanoseconds
        ):
            has_velocity = False

        result = MultiDOFJointTrajectoryPoint()
        result.time_from_start.nanosec = sample_time

        if not has_velocity:
            # Perform linear interpolation between points
            for i in range(len(start_point.transforms)):
                # Get the desired positions
                start_tf: Transform = start_point.transforms[i]  # type: ignore
                end_tf: Transform = end_point.transforms[i]  # type: ignore

                tfs = np.array(
                    [
                        controller_utils.convert_tf_to_array(start_tf),
                        controller_utils.convert_tf_to_array(end_tf),
                    ]
                ).T
                t = np.array([start_time.nanoseconds, end_time.nanoseconds])

                # Perform linear interpolation
                interpolation = interp1d(t, tfs, kind="linear")
                sample = interpolation(sample_time.nanoseconds)

                # Reconstruct the sample as a ROS message
                sample_msg = Transform()

                (
                    sample_msg.translation.x,
                    sample_msg.translation.y,
                    sample_msg.translation.z,
                ) = sample[:3]

                (
                    sample_msg.rotation.x,
                    sample_msg.rotation.y,
                    sample_msg.rotation.z,
                    sample_msg.rotation.w,
                ) = R.from_euler("xyz", sample[3:]).as_quat()

                result.transforms.append(sample_msg)  # type: ignore
        elif has_velocity:
            for i in range(len(start_point.transforms)):
                # Get the desired positions
                start_tf: Transform = start_point.transforms[i]  # type: ignore
                end_tf: Transform = end_point.transforms[i]  # type: ignore

                # Get the desired velocities
                start_vel = start_point.velocities[i]  # type: ignore
                end_vel = end_point.velocities[i]  # type: ignore

                tfs = np.array(
                    [
                        controller_utils.convert_tf_to_array(start_tf),
                        controller_utils.convert_tf_to_array(end_tf),
                    ]
                )
                vels = np.array(
                    [
                        controller_utils.convert_twist_to_array(start_vel),
                        controller_utils.convert_twist_to_array(end_vel),
                    ]
                )
                t = np.array([start_time.nanoseconds, end_time.nanoseconds])

                # Interpolate using a cubic hermite spline
                spline = CubicHermiteSpline(t, tfs, vels)
                sample_tf = spline(sample_time.nanoseconds)
                sample_vel = spline(sample_time.nanoseconds, 1)

                # Now reconstruct the TF message
                sample_tf_msg = Transform()

                (
                    sample_tf_msg.translation.x,
                    sample_tf_msg.translation.y,
                    sample_tf_msg.translation.z,
                ) = sample_tf[:3]

                (
                    sample_tf_msg.rotation.x,
                    sample_tf_msg.rotation.y,
                    sample_tf_msg.rotation.z,
                    sample_tf_msg.rotation.w,
                ) = R.from_euler("xyz", sample_tf[3:]).as_quat()

                # Reconstruct the velocity message
                sample_twist_msg = Twist()

                (
                    sample_twist_msg.linear.x,
                    sample_twist_msg.linear.y,
                    sample_twist_msg.linear.z,
                    sample_twist_msg.angular.x,
                    sample_twist_msg.angular.y,
                    sample_twist_msg.angular.z,
                ) = sample_vel

                # Finish up by saving the samples
                result.transforms.append(sample_tf_msg)  # type: ignore
                result.velocities.append(sample_twist_msg)  # type: ignore

        return result
