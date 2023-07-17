# angler

Angler is a collection of ROS 2 packages designed to support research and
development of underwater vehicle manipulator systems (UVMS).

## Main Features

The main features of the project include:

- Integration of [blue](https://github.com/evan-palmer/blue) for applying
custom localization and vehicle control algorithms (e.g., sliding mode control)
- Integration of [alpha](https://github.com/evan-palmer/alpha) for developing
custom manipulation algorithms for the [Reach Alpha 5 manipulator](https://reachrobotics.com/products/manipulators/reach-alpha/)
- ArduSub + Gazebo SITL integration including a simplified UVMS hydrodynamics
model for evaluating and testing the performance of your algorithms
- Support for developing custom planning algorithms
- Integration of set-based task priority inverse kinematic control (TPIK) for
whole-body control
- Energy optimal trajectory planning for operation in turbid conditions
