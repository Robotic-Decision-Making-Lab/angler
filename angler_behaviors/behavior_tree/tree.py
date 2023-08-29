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
from behavior_tree.behaviors.cleanup import make_on_mission_complete_behavior
from behavior_tree.behaviors.configure import make_setup_behavior
from behavior_tree.behaviors.mission import (
    make_execute_mission_behavior,
    make_save_start_mission_behavior,
)
from behavior_tree.primitives.arming import (
    make_block_on_disarm_behavior,
    make_save_armed_behavior,
)
from behavior_tree.primitives.planning import make_save_robot_state_behavior


def make_angler_tree() -> py_trees.behaviour.Behaviour:
    """Make a behavior tree that runs the full Angler system.

    Returns:
        A behavior tree that provides full-system autonomy.
    """
    root = py_trees.composites.Parallel(
        name="Angler Autonomy",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # Add the data gathering behaviors
    start_mission_key = "start_mission"
    arm_system_key = "arm_system"
    armed_key = "armed"
    robot_state_key = "robot_state"

    data_gathering = py_trees.composites.Sequence(
        name="Data gathering",
        memory=True,
        children=[
            make_save_start_mission_behavior(start_mission_key),
            make_save_armed_behavior(arm_system_key),
            make_save_robot_state_behavior(robot_state_key),
        ],
    )

    root.add_child(data_gathering)

    setup_finished_flag_key = "setup_finished"

    setup_and_execute_mission = py_trees.composites.Sequence(
        name="Setup and execute mission",
        memory=True,
        children=[
            make_setup_behavior(
                setup_finished_flag_key=setup_finished_flag_key, armed_key=armed_key
            ),
            make_execute_mission_behavior(
                start_mission_key=start_mission_key,
                robot_state_key=robot_state_key,
                planner_id="preplanned_end_effector_waypoint_planner",
                controller_id="tpik_joint_trajectory_controller",
            ),
            make_on_mission_complete_behavior(),
        ],
    )

    tasks = make_block_on_disarm_behavior(
        arm_system_key=arm_system_key,
        armed_key=armed_key,
        tasks=setup_and_execute_mission,
        on_disarm_behavior=py_trees.behaviours.SetBlackboardVariable(
            name="Setup needs to be redone",
            variable_name=setup_finished_flag_key,
            variable_value=False,
            overwrite=True,
        ),
    )

    root.add_child(tasks)

    return root


def main(args: list[str] | None = None):
    """Run the pre-planned waypoint planner."""
    rclpy.init(args=args)

    # Create the behavior tree
    root = make_angler_tree()
    tree = py_trees_ros.trees.BehaviourTree(root)

    # Setup the tree; this will throw if there is a timeout
    tree.setup(timeout=15.0)

    # Run the tree at a rate of 20hz
    tree.tick_tock(50)

    rclpy.spin(tree.node)  # type: ignore

    tree.shutdown()
    rclpy.shutdown()
