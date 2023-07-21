cmake_minimum_required(VERSION 3.8)
project(angler_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(action_files
  "action/PlanMission.action"
)

set(msg_files
  "msg/Waypoint.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs action_msgs nav_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
