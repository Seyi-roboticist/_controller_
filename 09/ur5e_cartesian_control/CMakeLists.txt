cmake_minimum_required(VERSION 3.8)
project(ur5e_cartesian_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ur_description REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(se3_sensor_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros_gz_sim QUIET)
find_package(ros2_control REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)

add_executable(two_positions_transform_monitor src/two_positions_transform_monitor.cpp)
ament_target_dependencies(two_positions_transform_monitor rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs)
install(TARGETS two_positions_transform_monitor DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY 
  launch
  config
  rviz
  world
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()