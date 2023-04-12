cmake_minimum_required(VERSION 3.8)
project(angler_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/ApplyForce.action"
  "action/CommandGripper.action"
  "action/FollowJointTrajectory.action"
  "action/FollowLinearTrajectory.action"
  "action/GetPickPoints.action"
  "action/GetPlacePoints.action"
  "action/MoveToJointPose.action"
  "action/MoveToLinearPose.action"
  "action/MoveUntilContact.action"
  "action/Stop.action"
  "action/GetTrajectory.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs action_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
