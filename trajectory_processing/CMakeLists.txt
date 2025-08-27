cmake_minimum_required(VERSION 3.18)
project(trajectory_processing)

###############################################################################
# 1. 의존성 설정 – 필요 시 추가
###############################################################################
find_package(ament_cmake      REQUIRED)   # 빌드 시스템
find_package(rclcpp           REQUIRED)   # ROS2 C++ API
find_package(nav_msgs         REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(Eigen3           REQUIRED)   # ### 수학 라이브러리 (선택)

###############################################################################
# 2. 포함 디렉터리 – 헤더 파일(.hpp) 넣을 위치
###############################################################################
include_directories(
  include
)

###############################################################################
# 3. C++ 라이브러리 / 노드
###############################################################################
# (1) 등시간격 보조 라이브러리
# add_library(time_param SHARED
#   src/time_param.cpp          # ### 구현 후 주석 해제
# )
# ament_target_dependencies(time_param
#   rclcpp nav_msgs geometry_msgs std_msgs Eigen3
# )

# (2) MPC 컨트롤러 실행 파일
# add_executable(mpc_controller
#   src/mpc_controller.cpp      # ### 구현 후 주석 해제
# )
# target_link_libraries(mpc_controller time_param)
# ament_target_dependencies(mpc_controller
#   rclcpp nav_msgs geometry_msgs std_msgs
# )

###############################################################################
# 4. Python 스크립트 설치
# scripts/ 폴더 안에 *.py 노드를 추가하면 여기 나열
###############################################################################
install(PROGRAMS
    scripts/traj_time_parameterizer.py
  DESTINATION lib/${PROJECT_NAME}
)

###############################################################################
# 5. 런치·데이터·헤더 설치
###############################################################################
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

###############################################################################
# 6. ament 패키지 내보내기
###############################################################################
ament_export_include_directories(include)
ament_export_libraries(time_param)
ament_package()
