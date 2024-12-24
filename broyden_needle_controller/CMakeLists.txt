cmake_minimum_required(VERSION 3.8)
project(broyden_needle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(broyden_needle_controller SHARED
  src/broyden_controller.cpp
)

target_compile_features(broyden_needle_controller PUBLIC cxx_std_17)
target_include_directories(broyden_needle_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/needle_controllers>
)

ament_target_dependencies(broyden_needle_controller
  Eigen3
  rclcpp
  pluginlib
  control_msgs
  controller_interface
  geometry_msgs
)

pluginlib_export_plugin_description_file(controller_interface broyden_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/broyden_needle_controller
)
install(
  TARGETS broyden_needle_controller
  EXPORT export_velocity_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()