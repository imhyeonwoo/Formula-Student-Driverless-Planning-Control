# ──────────────────────────────────────────────
# cones_ukf/CMakeLists.txt
# ── 새 노드를 추가할 때 건드려야 할 곳마다 ★주석★ 표시 ──
cmake_minimum_required(VERSION 3.18)
project(cones_no_color)

# ---- C++ 표준 ----
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---- 패키지 의존성 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)                # ★ 파이썬 노드 있을 때도 선언
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interface REQUIRED)     # TrackedConeArray 가 들어있는 패키지

# ============================================================
# 1. C++ 노드 (예: visualize_cones_cpp.cpp)를 추가하고 싶다면
#    ① add_executable / ② ament_target_dependencies / ③ install(TARGETS)에 등록
# ============================================================

# ★새로운 C++ 노드 예시★
# add_executable(visualize_cones_cpp src/visualize_cones_cpp.cpp)
# ament_target_dependencies(visualize_cones_cpp
#   rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs custom_interface)

# -------------------- Python 노드 --------------------
install(PROGRAMS
  scripts/visualize_cones.py          # ★새 .py 파일 추가 시 여기에 열거
  scripts/reference_path_planning.py
  scripts/cone_safe_zone.py
  DESTINATION lib/${PROJECT_NAME})

# -------------------- C++ 바이너리 설치 ----------------
install(TARGETS
  # visualize_cones_cpp               # ★위 add_executable() 했으면 여기도 추가
  DESTINATION lib/${PROJECT_NAME})

# -------------------- 기타 리소스(launch, rviz 등) -----
install(DIRECTORY
  launch                              # launch/… 없으면 제거해도 무방
  DESTINATION share/${PROJECT_NAME})

ament_package()
