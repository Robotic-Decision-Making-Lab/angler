cmake_minimum_required(VERSION 3.8)
project(angler_bringup)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  ament_cmake
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
