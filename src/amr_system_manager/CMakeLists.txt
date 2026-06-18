cmake_minimum_required(VERSION 3.8)
project(amr_system_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(system_manager_node src/system_manager_node.cpp)
target_compile_features(system_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(system_manager_node
  amr_interfaces
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS system_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
