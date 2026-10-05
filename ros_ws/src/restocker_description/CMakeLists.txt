# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_description VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Launch tests lease a ROS domain and Gazebo partition at run time so concurrent runs
  # do not see each other's nodes. See cmake/restocker_test_isolation.cmake.
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/restocker_test_isolation.cmake)

  ament_add_pytest_test(
    test_robot_description
    test/test_robot_description.py
    ENV RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_workcell_description
    test/test_workcell_description.py
    ENV RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
  )
  add_launch_test(test/test_robot_tf.py TIMEOUT 60)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
