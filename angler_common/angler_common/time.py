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

from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.time import Time


def add_ros_time_duration_msg(time: Time, duration: DurationMsg) -> Time:
    """Add a ROS Time and a Duration message together to create a new Time.

    Args:
        time: The Time addend.
        duration: The Duration message addend.

    Returns:
        The resulting sum as a new Time object.
    """
    duration_nanoseconds = duration.sec * 10**9 + duration.nanosec
    return Time(
        nanoseconds=time.nanoseconds + duration_nanoseconds, clock_type=time.clock_type
    )
