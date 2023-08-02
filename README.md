# Angler :fish:

Angler is a general ROS 2 framework designed to support research, development,
and deployment of underwater vehicle manipulator systems (UVMS).

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

If you have questions regarding usage of the BlueROV2 driver or regarding
contributing to this project, please ask a question on our
[Discussions](https://github.com/evan-palmer/angler/discussions) board!

## License

Angler is released under the MIT license.
