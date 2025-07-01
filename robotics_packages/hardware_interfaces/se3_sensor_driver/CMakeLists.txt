cmake_minimum_required(VERSION 3.8)
project(se3_sensor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hardware_interface REQUIRED)

# Include directories 
include_directories(include)

# Adding the library
add_library(se3_sensor_plugin SHARED src/hardware_interface.cpp)
ament_target_dependencies(
  se3_sensor_plugin 
  rclcpp
  controller_manager controller_manager_msgs
  hardware_interface 
  pluginlib
  geometry_msgs
)

# Add the TF lookup server executable
add_executable(tf_lookup_server src/tf_lookup_server.cpp)
ament_target_dependencies(
  tf_lookup_server
  rclcpp
  tf2_ros
  geometry_msgs
)

# Install the TF lookup server
install(TARGETS
  tf_lookup_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  se3_sensor_plugin DESTINATION lib
)
pluginlib_export_plugin_description_file(hardware_interface se3_sensor_plugin.xml)

install(DIRECTORY 
  urdf launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
