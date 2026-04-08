cmake_minimum_required(VERSION 3.8)
project(plc_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

# PLC Hardware Interface plugin (SystemInterface)
add_library(plc_ros2_bridge_plugin SHARED
  src/plc_hardware_interface.cpp
  src/ethernet_ip_interface.cpp
  src/opcua_interface.cpp
)
ament_target_dependencies(plc_ros2_bridge_plugin
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface plc_ros2_bridge_plugin.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS plc_ros2_bridge_plugin
  DESTINATION lib
)

# Install config, urdf, and documentation
install(DIRECTORY
  config
  urdf
  doc
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(plc_ros2_bridge_plugin)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()
