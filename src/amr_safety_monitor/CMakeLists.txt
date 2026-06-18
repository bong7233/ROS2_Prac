cmake_minimum_required(VERSION 3.8)
project(amr_safety_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(safety_monitor_node src/safety_monitor_node.cpp)
target_compile_features(safety_monitor_node PUBLIC cxx_std_17)
ament_target_dependencies(safety_monitor_node
  amr_interfaces
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
