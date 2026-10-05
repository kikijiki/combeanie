# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_planning_probe VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(planning_probe src/planning_probe.cpp)
target_compile_options(planning_probe PRIVATE -Wall -Wextra)
ament_target_dependencies(planning_probe
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_msgs
  geometric_shapes
  urdf
  tf2_eigen)
target_link_libraries(planning_probe ${YAML_CPP_LIBRARIES})

install(TARGETS planning_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
