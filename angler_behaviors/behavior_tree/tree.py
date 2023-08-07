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

import py_trees
import py_trees_ros
import rclpy


def make_angler_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Angler Autonomy",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    return root


def main(args: list[str] | None = None):
    """Run the pre-planned waypoint planner."""
    rclpy.init(args=args)

    # Create the behavior tree
    root = make_angler_tree()
    tree = py_trees_ros.trees.BehaviourTree(root)

    # Setup the tree; this will throw if there is a timeout
    tree.setup(timeout=5.0)

    # Run the tree at a rate of 20hz
    tree.tick_tock(50)

    tree.shutdown()
    rclpy.shutdown()
