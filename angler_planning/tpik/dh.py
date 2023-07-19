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

import json
import os
from copy import deepcopy

import numpy as np


class DenavitHartenburgParams:
    """Container for interacting with Denavit-Hartenburg (DH) parameters."""

    def __init__(self, params: list[dict[str, float]]) -> None:
        """Create a new DH parameters interface.

        Args:
            params: The DH parameters to store. Each set of parameters should include:
                "a", "alpha", "d", and "theta".
        """
        self.params = self._create_array_from_dict(params)

    def __call__(self, joint_angles: np.ndarray) -> np.ndarray:
        """Calculate the updated DH params using the current joint angles.

        Args:
            joint_angles: The current joint angles. These should be provided in the
                same order as the DH table specification.

        Returns:
            The current DH parameters.
        """
        updated_params = deepcopy(self.params)
        updated_params[3] = updated_params[3] + joint_angles

        return updated_params

    @staticmethod
    def _create_array_from_dict(params: list[dict[str, float]]) -> np.ndarray:
        """Create a numpy array from the parameters.

        Args:
            params: The DH parameters to convert.

        Returns:
            The DH table as a numpy array.
        """
        ar = np.zeros((len(params), 4))

        for i, joint in enumerate(params):
            ar[i] = np.array([joint["a"], joint["alpha"], joint["d"], joint["theta"]])

        return ar


def load_dh_params_from_json(fp: str) -> DenavitHartenburgParams:
    """Load the Denavit-Hartenburg Parameters from a JSON file.

    Args:
        fp: The full path to the JSON parameters file.

    Returns:
        The Denavit-Hartenburg parameters.
    """
    if not (fp.endswith(".json") and os.path.isfile(fp)):
        raise ValueError(
            "The provided file is invalid. Please ensure that the configurations are"
            " defined correctly and that the path exists."
        )

    with open(fp, encoding="utf-8") as file:
        return DenavitHartenburgParams(**json.load(file))
