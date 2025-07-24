cmake_minimum_required(VERSION 3.8)
project(reference_path_classifier)

# ─────────────────────────────────────────
# Global settings
# ─────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -march=native -ffast-math)
endif()

# ─────────────────────────────────────────
# Dependencies
# ─────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_interface REQUIRED)
find_package(OpenMP REQUIRED)

# nanoflann (header‑only)
find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp)
if(NANOFLANN_INCLUDE_DIR)
  message(STATUS "nanoflann include dir: ${NANOFLANN_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "nanoflann.hpp not found. Install libnanoflann-dev or add include path.")
endif()

# ─────────────────────────────────────────
# Python script install
# ─────────────────────────────────────────
install(
  PROGRAMS scripts/classify_cones_by_side.py
  DESTINATION lib/${PROJECT_NAME})

# include / launch install (optional)
install(
  DIRECTORY include/
  DESTINATION include/)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ─────────────────────────────────────────
# C++ executable
# ─────────────────────────────────────────
add_executable(classify_cones_by_side src/classify_cones_by_side.cpp)

target_include_directories(classify_cones_by_side PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${NANOFLANN_INCLUDE_DIR})

ament_target_dependencies(classify_cones_by_side
  rclcpp tf2_ros geometry_msgs visualization_msgs
  custom_interface Eigen3)

target_link_libraries(classify_cones_by_side OpenMP::OpenMP_CXX)

install(TARGETS classify_cones_by_side DESTINATION lib/${PROJECT_NAME})

ament_package()
