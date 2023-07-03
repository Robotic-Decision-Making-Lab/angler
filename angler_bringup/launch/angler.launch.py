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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to run the Angler system.

    Returns:
        The Angler ROS 2 launch description.
    """
    # Declare the launch arguments
    args = [
        DeclareLaunchArgument(
            "description_package",
            default_value="angler_description",
            description=(
                "The description package with the angler configuration files. This is"
                " typically not set, but is available in case another description"
                " package has been defined."
            ),
        ),
        DeclareLaunchArgument(
            "planning_file",
            default_value="planning.yaml",
            description="The Angler planning configuration file.",
        ),
        DeclareLaunchArgument(
            "mux_file",
            default_value="mux.yaml",
            description="The Angler mux + demux configuration file.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="blue_controllers.yaml",
            description="The BlueROV2 Heavy controller configuration file.",
        ),
        DeclareLaunchArgument(
            "localization_file",
            default_value="localization.yaml",
            description="The BlueROV2 Heavy localization configuration file.",
        ),
        DeclareLaunchArgument(
            "manager_file",
            default_value="blue_manager.yaml",
            description="The BlueROV2 Heavy manager configuration file.",
        ),
        DeclareLaunchArgument(
            "mavros_file",
            default_value="mavros.yaml",
            description="The MAVROS configuration file.",
        ),
        DeclareLaunchArgument(
            "base_controller",
            default_value="ismc",
            description=(
                "The ROV controller to use; this should be the same name as the"
                " controller's executable."
            ),
            choices=["ismc"],
        ),
        DeclareLaunchArgument(
            "manipulator_controller",
            default_value="forward_velocity_controller",
            description="The ros2_control controller to use with the manipulator(s).",
        ),
        DeclareLaunchArgument(
            "localization_source",
            default_value="gazebo",
            choices=["mocap", "camera", "gazebo"],
            description="The localization source to stream from.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description=(
                "Launch the BlueROV2 camera stream. This is automatically set to true"
                " when using the camera for localization."
            ),
        ),
        DeclareLaunchArgument(
            "use_mocap",
            default_value="false",
            description=(
                "Launch the Qualisys motion capture stream. This is automatically"
                " set to true when using the motion capture system for localization."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="angler.world",
            description="The world configuration to load if using Gazebo.",
        ),
        DeclareLaunchArgument(
            "ardusub_params_file",
            default_value="angler.parm",
            description=(
                "The ArduSub parameters that the BlueROV2 should use if running in"
                " simulation."
            ),
        ),
        DeclareLaunchArgument(
            "manipulator_serial_port",
            default_value="''",
            description=(
                "The serial port that the Alpha 5 is available at (e.g., /dev/ttyUSB0)."
            ),
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    planning_file = LaunchConfiguration("planning_file")
    mux_file = LaunchConfiguration("mux_file")
    controllers_file = LaunchConfiguration("controllers_file")
    localization_file = LaunchConfiguration("localization_file")
    manager_file = LaunchConfiguration("manager_file")
    mavros_file = LaunchConfiguration("mavros_file")

    base_controller = LaunchConfiguration("base_controller")
    manipulator_controller = LaunchConfiguration("manipulator_controller")
    localization_source = LaunchConfiguration("localization_source")
    use_camera = LaunchConfiguration("use_camera")
    use_mocap = LaunchConfiguration("use_mocap")
    use_sim = LaunchConfiguration("use_sim")
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")
    ardusub_params_file = LaunchConfiguration("ardusub_params_file")
    manipulator_serial_port = LaunchConfiguration("manipulator_serial_port")

    # Declare additional launch files to run
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("angler_planning"), "planning.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", planning_file]
                )
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("angler_mux"), "mux.launch.py"])
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", mux_file]
                )
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_bringup"), "bluerov2_heavy.launch.py"]
                )
            ),
            launch_arguments={
                "blue_description": description_package,
                "controllers_file": controllers_file,
                "localization_file": localization_file,
                "mavros_file": mavros_file,
                "manager_file": manager_file,
                "controller": base_controller,
                "localization_source": localization_source,
                "use_camera": use_camera,
                "use_mocap": use_mocap,
                "ardusub_params_file": ardusub_params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("alpha_bringup"), "alpha.launch.py"]
                )
            ),
            launch_arguments={
                "robot_controller": manipulator_controller,
                "serial_port": manipulator_serial_port,
                "use_rviz": PythonExpression("False"),
                "use_fake_hardware": use_sim,
            }.items(),
        ),
    ]

    return LaunchDescription(args + includes)
