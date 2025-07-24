cmake_minimum_required(VERSION 3.5)
project(cone_labeling)

# C++17, warnings
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(custom_interface REQUIRED)       # ← 추가

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 1) cone_classifier
add_executable(cone_classifier src/cone_classifier.cpp)
ament_target_dependencies(cone_classifier
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  custom_interface                              # ← 추가
)

# 2) cone_delaunay_connector
add_executable(cone_delaunay_connector src/cone_delaunay_connector.cpp)
ament_target_dependencies(cone_delaunay_connector
  rclcpp visualization_msgs geometry_msgs std_msgs sensor_msgs cv_bridge OpenCV
)

# 3) midpoint_bspline_interpolator
add_executable(midpoint_bspline_interpolator src/midpoint_bspline_interpolator.cpp)
ament_target_dependencies(midpoint_bspline_interpolator
  rclcpp visualization_msgs geometry_msgs std_msgs sensor_msgs
)

# 설치: 실행파일과 launch 파일
install(TARGETS
  cone_classifier
  cone_delaunay_connector
  midpoint_bspline_interpolator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

