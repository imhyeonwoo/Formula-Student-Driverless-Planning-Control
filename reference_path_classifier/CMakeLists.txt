cmake_minimum_required(VERSION 3.8)
project(reference_path_classifier)

# ────────────────────────────────────────────────────
# 기본 설정
# ────────────────────────────────────────────────────
# C++17 (추후 C++ 노드 작성 시 사용)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ────────────────────────────────────────────────────
# 의존성 찾기
# ────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)               # Python 노드
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(custom_interface REQUIRED)    # 사용자 정의 메시지 패키지

# C++ 노드 추가 시 예시
# find_package(rclcpp REQUIRED)

# ────────────────────────────────────────────────────
# Python 스크립트 설치
# ────────────────────────────────────────────────────
install(
  PROGRAMS
    scripts/classify_cones_by_side.py
  DESTINATION lib/${PROJECT_NAME}
)

# ────────────────────────────────────────────────────
# 헤더 / 런치 파일 설치 (선택 사항)
# ────────────────────────────────────────────────────
# include/ 안의 헤더들을 나중에 쓸 수 있도록 설치
install(
  DIRECTORY include/
  DESTINATION include/
)

# launch/ 디렉터리 통째로 설치
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ────────────────────────────────────────────────────
# (옵션) C++ 노드 빌드 예시 – src/에 소스 추가 시 활성화
# ────────────────────────────────────────────────────
# add_executable(${PROJECT_NAME}_node src/your_cpp_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp tf2_ros visualization_msgs builtin_interfaces custom_interface
# )
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ────────────────────────────────────────────────────
ament_package()
