cmake_minimum_required(VERSION 3.18)
project(speed_planning)

if(NOT CMAKE_CXX_STANDARD)  # C++17
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(speed_planner src/speed_planner.cpp)
target_include_directories(speed_planner PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(speed_planner
  rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs Eigen3)

add_executable(global_speed_planner src/global_speed_planner.cpp)
ament_target_dependencies(global_speed_planner 
  rclcpp std_msgs geometry_msgs visualization_msgs ament_index_cpp)

add_executable(speed_planner_preview src/speed_planner_preview.cpp)
ament_target_dependencies(speed_planner_preview
  rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs)

add_executable(gps_speed_node src/gps_speed_node.cpp)
ament_target_dependencies(gps_speed_node 
  rclcpp sensor_msgs std_msgs)

add_executable(current_speed_from_odom_node src/current_speed_from_odom.cpp)
ament_target_dependencies(current_speed_from_odom_node
    rclcpp std_msgs nav_msgs
)

install(TARGETS 
  speed_planner
  global_speed_planner
  speed_planner_preview
  gps_speed_node 
  current_speed_from_odom_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
