# Angler :fish:

Angler is a general ROS 2 framework designed to support research, development,
and deployment of underwater vehicle manipulator systems (UVMS).

## ⚠️ Notice ⚠️

We have made a number of major improvements and modifications to this project
since its initial release. **All features that have been implemented in this
project still exists and have been greatly improved.** Angler as a framework
is now split into the following distinct repositories:

* [blue](https://github.com/Robotic-Decision-Making-Lab/blue): Tutorials and
  complete integration examples have been moved to this project to consolidate
  our documentation.
* [auv_controllers](https://github.com/Robotic-Decision-Making-Lab/auv_controllers): All
  controllers in this project have been refactored using ros2_control to improve
  generalization and to simplify integration of custom algorithms. The whole-body
  controllers, including the TPIK controller, have been moved here. The dependency
  on ArduSub has also been abstracted away so that these algorithms can be applied
  to a wider variety of systems.
* [hydrodynamics](https://github.com/Robotic-Decision-Making-Lab/hydrodynamics): This
  repository implements an API for parsing hydrodynamic parameters from a URDF and
  for using the parsed parameters for forward/inverse dynamics.
* [ardusub_driver](https://github.com/Robotic-Decision-Making-Lab/ardusub_driver): This
  repository hosts the ArduSub integration implementation, including a hardware
  interface for direct thruster control that integrates with `auv_controllers`.
* [reach](https://github.com/Robotic-Decision-Making-Lab/reach): The Reach Alpha 5
  implementation has been generalized to better support integration with additional
  Reach manipulator projects, such as the Reach Bravo 7. The project has also been
  improved to enable integration with rigid body dynamics libraries for use in MPC
  implementations.
* Sensor drivers that were previously implemented in this project and their integration
  with state estimators are now described in the blue documentation linked above. For
  example, we have integrated support for the Water Linked DVL-A50 and Water Linked
  Underwater GPS.

## Main Features

The main features of the project include:

- Support for developing custom whole-body controllers alongside the existing
  control algorithms like Set-Based Task-Priority Inverse Kinematic (TPIK)
  control
- Support for developing custom trajectory and motion planning algorithms
- ArduSub + Gazebo SITL integration for evaluating the performance of your
  system in a simulation environment
- RViz2 integration for visualization
- Modular design to enable integration of personalized platforms
- Integration of [Blue](https://github.com/evan-palmer/blue) to provide support
  for custom vehicle controllers like sliding-mode control

## Installation

Angler is currently supported on Linux, and is available for the ROS
distributions Humble, Iron, and Rolling. To install Angler, first clone this
project to the `src` directory of your ROS workspace, replacing `$ROS_DISTRO`
with the desired ROS distribution or `main` for Rolling:

```bash
git clone -b $ROS_DISTRO git@github.com:evan-palmer/angler.git
```

After cloning the project, install all external dependencies using `vcs`:

```bash
vcs import src < src/angler/angler.repos
```

Finally, install the ROS dependencies using `rosdep`, again, replacing
`$ROS_DISTRO` with the desired ROS distribution:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## Quick start

ROS 2 launch files have been provided to start the Angler system. To launch
the SITL interface for the BlueROV2 Heavy with a Reach Alpha 5 manipulator, run

```bash
ros2 launch angler_bringup bluerov2_heavy_alpha.launch.py use_sim:=true
```

A full description of the launch arguments and their respective default values
can be obtained by running the following command:

```bash
ros2 launch angler_bringup bluerov2_heavy_alpha.launch.py --show-args
```

## Getting help

If you have questions regarding usage of Angler or regarding contributing to
this project, please ask a question on our [Discussions](https://github.com/evan-palmer/angler/discussions)
board!

## License

Angler is released under the MIT license.
