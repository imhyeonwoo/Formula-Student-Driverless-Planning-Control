cmake_minimum_required(VERSION 3.15)
project(wheel_trajectory)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(wheel_trajectory_node src/wheel_trajectory_node.cpp)
ament_target_dependencies(wheel_trajectory_node
  rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)

install(TARGETS wheel_trajectory_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
