cmake_minimum_required(VERSION 3.8)
project(amr_io_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mock_io_driver_node src/mock_io_driver_node.cpp)
target_compile_features(mock_io_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(mock_io_driver_node
  amr_interfaces
  diagnostic_msgs
  diagnostic_updater
  rclcpp
)

install(TARGETS mock_io_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
