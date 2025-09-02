cmake_minimum_required(VERSION 3.18)
project(speed_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# [추가] Matplot++는 선택사항 (없으면 CSV만 출력)
find_package(Matplot++ QUIET)

include_directories(include)

add_executable(simple_speed_planner src/simple_speed_planner.cpp)
ament_target_dependencies(simple_speed_planner
  rclcpp std_msgs nav_msgs geometry_msgs)

add_executable(stcap_fb_speed_planner src/stcap_fb_speed_planner_node.cpp)
ament_target_dependencies(stcap_fb_speed_planner 
  rclcpp std_msgs nav_msgs)

# [추가] 디버그용 실행파일: 종료 시 CSV/PNG 저장
add_executable(simple_speed_planner_debug src/simple_speed_planner_debug.cpp)
ament_target_dependencies(simple_speed_planner_debug rclcpp std_msgs nav_msgs)

# [추가] Matplot++ 발견 시 링크 + 플래그 정의
if(Matplot++_FOUND)
  target_link_libraries(simple_speed_planner_debug Matplot++::matplot)
  target_compile_definitions(simple_speed_planner_debug PRIVATE HAVE_MATPLOT=1)
endif()

add_executable(curvature_speed_planner_node src/curvature_speed_planner_node.cpp)
ament_target_dependencies(curvature_speed_planner_node
  rclcpp
  nav_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  simple_speed_planner
  stcap_fb_speed_planner
  simple_speed_planner_debug
  curvature_speed_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
