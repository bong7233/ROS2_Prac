cmake_minimum_required(VERSION 3.8)
project(amr_operator_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(amr_operator_ui_node
  include/amr_operator_ui/main_window.hpp
  include/amr_operator_ui/robot_scene_widget.hpp
  include/amr_operator_ui/robot_state_cache.hpp
  include/amr_operator_ui/ros_worker.hpp
  src/main.cpp
  src/main_window.cpp
  src/robot_scene_widget.cpp
  src/ros_worker.cpp
)
target_compile_features(amr_operator_ui_node PUBLIC cxx_std_17)
target_include_directories(amr_operator_ui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(amr_operator_ui_node
  amr_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
)
target_link_libraries(amr_operator_ui_node Qt6::Widgets)

install(TARGETS amr_operator_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
