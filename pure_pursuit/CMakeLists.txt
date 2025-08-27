cmake_minimum_required(VERSION 3.8)
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

add_executable(pure_pursuit_static 
  src/pure_pursuit_static.cpp
)
ament_target_dependencies(pure_pursuit_static
  rclcpp std_msgs nav_msgs visualization_msgs
)

add_executable(pure_pursuit_dynamic 
  src/pure_pursuit_dynamic.cpp
)
ament_target_dependencies(pure_pursuit_dynamic
  rclcpp std_msgs nav_msgs visualization_msgs
)

add_executable(pure_pursuit_adaptive_node
  src/pure_pursuit_adaptive.cpp
)
ament_target_dependencies(pure_pursuit_adaptive_node
  rclcpp std_msgs nav_msgs visualization_msgs
)

install(TARGETS 
  pure_pursuit_static
  pure_pursuit_dynamic
  pure_pursuit_adaptive_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
