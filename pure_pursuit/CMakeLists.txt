cmake_minimum_required(VERSION 3.18)
project(pure_pursuit)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 1) static look-ahead distance pure pursuit
add_executable(pure_pursuit_static 
  src/pure_pursuit_static.cpp
)
ament_target_dependencies(pure_pursuit_static
  rclcpp std_msgs nav_msgs visualization_msgs
)

# 2) dynamic look-ahead distance pure pursuit
add_executable(pure_pursuit_dynamic 
  src/pure_pursuit_dynamic.cpp
)
ament_target_dependencies(pure_pursuit_dynamic
  rclcpp std_msgs nav_msgs visualization_msgs
)

# 3) adaptive look-ahead distance pure pursuit
add_executable(pure_pursuit_adaptive_node
  src/pure_pursuit_adaptive.cpp
)
ament_target_dependencies(pure_pursuit_adaptive_node
  rclcpp std_msgs nav_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

# 4) predictive pure pursuit
add_executable(pure_pursuit_predictive_node 
  src/pure_pursuit_predictive.cpp
)
ament_target_dependencies(pure_pursuit_predictive_node
  rclcpp std_msgs nav_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

install(TARGETS 
  pure_pursuit_static
  pure_pursuit_dynamic
  pure_pursuit_adaptive_node
  pure_pursuit_predictive_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
