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

import operator

import py_trees
from behavior_tree.primitives.arming import make_system_arming_behavior


def make_setup_behavior(setup_finished_flag_key: str) -> py_trees.behaviour.Behaviour:
    """Make a behavior that sets up the system prior to beginning a mission.

    Args:
        setup_finished_flag_key: The key at which the setup flag is stored.

    Returns:
        A system setup behavior.
    """
    # Don't repeat setup if we have already done it
    check_setup_finished = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Setup complete?",
        check=py_trees.common.ComparisonExpression(
            variable=setup_finished_flag_key, value=True, operator=operator.eq
        ),
    )

    # Once we have a plan, arm so that we can start trajectory tracking
    arming = make_system_arming_behavior(arm=True, use_passthrough_mode=True)

    # Finish up by indicating that the setup has finished
    finished_setup = py_trees.behaviours.SetBlackboardVariable(
        "Setup finished!", setup_finished_flag_key, True, overwrite=True
    )

    setup = py_trees.composites.Sequence(
        name="Setup the system for a mission",
        memory=True,
        children=[arming, finished_setup],
    )

    return py_trees.composites.Selector(
        name="Run system setup", memory=False, children=[check_setup_finished, setup]
    )
