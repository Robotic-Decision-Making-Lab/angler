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

import numpy as np

"""
We are not scaling the waves, we are distorting them

we can only match the horizontal wave kinematics, but not the vertical (not orbital
velocities) - vertical accuracy can be assessed

Trying to have matching wave conditions

In lab we cannot reproduce conditions at full scale

Normally, reduce scale to reproduce the waves at some scale, need to scale down auv

Consider the condition that has the dominant effect on the AUV (horizontal velocity)

Need to keep one thing constant (wave period) - match horizontal velocity

In the distortion, the orbital velocities will be distorted

If we say that we are scaling, we need to be able to say that we are scaling the dynamics
and kinematics of the real world

Assuming that there is only one component that is reproducible and we are going to apply
that, the rest will be distorted
"""


class Constraint:
    def __init__(self, jacobian: np.ndarray, gain: float, priority: float) -> None:
        self.jacobian = jacobian
        self.gain = gain
        self.priority = priority


class EqualityConstraint(Constraint):
    def __init__(self, jacobian: np.ndarray, gain: float, priority: float) -> None:
        super().__init__(jacobian, gain, priority)


class SetConstraint(Constraint):
    def __init__(
        self,
        jacobian: np.ndarray,
        range: tuple[float | None, float | None],
        gain: float,
        priority: float,
    ) -> None:
        super().__init__(jacobian, gain, priority)
