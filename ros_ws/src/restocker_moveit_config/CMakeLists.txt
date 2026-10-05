# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_moveit_config VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py
    ENV
      RESTOCKER_CONTROL_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_control
      RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
  )
endif()

ament_package()
