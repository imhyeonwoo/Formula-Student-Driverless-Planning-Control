cmake_minimum_required(VERSION 3.5)
project(gps_global_planner)

# ──────────────── 빌드 옵션 ────────────────
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ──────────────── 의존성 ────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(custom_interface REQUIRED)        # ROI 메시지
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

# ──────────────── C++ 노드 ────────────────
add_executable(gps_to_local_cartesian_node src/gps_to_local_cartesian.cpp)
ament_target_dependencies(
  gps_to_local_cartesian_node
  rclcpp sensor_msgs geometry_msgs)

add_executable(local_cartesian_path_publisher_node src/local_cartesian_path_publisher.cpp)
ament_target_dependencies(
  local_cartesian_path_publisher_node
  rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(vehicle_tf_broadcaster_node src/vehicle_tf_broadcaster.cpp)
ament_target_dependencies(
  vehicle_tf_broadcaster_node
  rclcpp geometry_msgs std_msgs tf2 tf2_ros)

add_executable(status_colored_path_publisher_node src/status_colored_path_publisher.cpp)
ament_target_dependencies(status_colored_path_publisher_node
  rclcpp geometry_msgs std_msgs visualization_msgs
  ament_index_cpp)

install(TARGETS
  gps_to_local_cartesian_node
  local_cartesian_path_publisher_node
  vehicle_tf_broadcaster_node
  status_colored_path_publisher_node
  DESTINATION lib/${PROJECT_NAME})
  

# ──────────────── Python 노드 ────────────────
install(PROGRAMS
  scripts/auto_place_cones.py
  scripts/cone_roi_publisher.py
  scripts/course_csv_creator.py
  scripts/global_yaw_estimator_node.py
  scripts/publish_global_cones.py
  DESTINATION lib/${PROJECT_NAME})

# ──────────────── 런치 / 데이터 ────────────────
install(DIRECTORY launch data
  DESTINATION share/${PROJECT_NAME})

ament_package()
