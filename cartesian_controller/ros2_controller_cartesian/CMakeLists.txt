cmake_minimum_required(VERSION 3.8)
project(ros2_controller_cartesian)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(ros2_controller_cartesian_plugin SHARED src/cartesian_controller.cpp)
ament_target_dependencies(
  ros2_controller_cartesian_plugin
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
  kdl_parser
  realtime_tools
  tf2_ros
  tf2_kdl
  tf2_eigen
  tf2_geometry_msgs
  Eigen3
)

pluginlib_export_plugin_description_file(controller_interface ros2_controller_cartesian_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  ros2_controller_cartesian_plugin 
  DESTINATION lib
)


ament_package()
