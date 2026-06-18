cmake_minimum_required(VERSION 3.8)
project(amr_base_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(diff_drive_base_controller_node src/diff_drive_base_controller_node.cpp)
target_compile_features(diff_drive_base_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(diff_drive_base_controller_node
  amr_interfaces
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

install(TARGETS diff_drive_base_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
