cmake_minimum_required(VERSION 3.15)
project(local_speed_planner)

# 기본 C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)              # Python 노드 실행용
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# ----- C++ 노드 (추후 필요시) -----
# add_executable(speed_planner_node src/speed_planner.cpp)
# ament_target_dependencies(speed_planner_node
#   rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs)
# install(TARGETS speed_planner_node
#   DESTINATION lib/${PROJECT_NAME})

# ----- Python 노드 설치 -----
install(PROGRAMS
  scripts/path_sampler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
