cmake_minimum_required(VERSION 3.8)
project(speed_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(simple_speed_planner src/simple_speed_planner.cpp)
ament_target_dependencies(simple_speed_planner
  rclcpp std_msgs nav_msgs geometry_msgs)

add_executable(stcap_fb_speed_planner src/stcap_fb_speed_planner_node.cpp)
ament_target_dependencies(stcap_fb_speed_planner 
  rclcpp std_msgs nav_msgs)

install(TARGETS
  simple_speed_planner
  stcap_fb_speed_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
