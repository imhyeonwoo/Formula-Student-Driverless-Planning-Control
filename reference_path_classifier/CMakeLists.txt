cmake_minimum_required(VERSION 3.8)
project(reference_path_classifier)

# ────────────────────────────────────────────────────
# 공통 설정
# ────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ────────────────────────────────────────────────────
# 패키지 찾기
# ────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

# Python 노드
find_package(rclpy REQUIRED)

# C++ 노드
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_interface REQUIRED)

# ────────────────────────────────────────────────────
# Python 스크립트 설치
# ────────────────────────────────────────────────────
install(
  PROGRAMS
    scripts/classify_cones_by_side.py
  DESTINATION lib/${PROJECT_NAME}
)

# ────────────────────────────────────────────────────
# (선택) include·launch 설치
# ────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ────────────────────────────────────────────────────
# C++ 노드 빌드
# ────────────────────────────────────────────────────
add_executable(classify_cones_by_side
  src/classify_cones_by_side.cpp)

# Eigen 헤더 포함 경로
target_include_directories(classify_cones_by_side PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(classify_cones_by_side
  rclcpp visualization_msgs geometry_msgs
  custom_interface tf2_ros Eigen3)

install(
  TARGETS classify_cones_by_side
  DESTINATION lib/${PROJECT_NAME})

# ────────────────────────────────────────────────────
ament_package()
