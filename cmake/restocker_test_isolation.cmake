# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Route every launch test, gtest and pytest through a runner that leases its ROS domain and Gazebo
# partition.
#
# A fixed `ROS_DOMAIN_ID` or `GZ_PARTITION` is identical across workspaces, so two worktrees
# running the same test see each other's nodes, and a `kill -9`'d run leaves nodes on a domain the
# next run reuses.
#
# Plain gtests need it as much as launch tests do. A gtest that creates nodes otherwise runs on
# the shell's domain 0, where the same test built in another worktree runs at the same time. The
# two harnesses then answer each other's actions and read each other's topics. That failure only
# looks like a load flake because the machine is loaded exactly when several agents run package
# suites at once.
#
# This shadows `add_launch_test`, `ament_add_gtest` and `ament_add_pytest_test` so isolation
# cannot be forgotten when a test is added. `_add_launch_test`, `_ament_add_gtest` and
# `_ament_add_pytest_test` are the upstream definitions, which CMake preserves under those names
# when a command is redefined.
#
# Include after `find_package(launch_testing_ament_cmake REQUIRED)`, and after
# `find_package(ament_cmake_gtest)` / `find_package(ament_cmake_pytest)` in a package that has
# those tests. A shadow can only replace a command that already exists, so the wrong order would
# leave that kind of test unleased. The check deferred to the end of the directory turns that
# into a configure error instead.

set(RESTOCKER_TEST_RUNNER "${CMAKE_CURRENT_LIST_DIR}/../scripts/run_isolated_test.py")
if(NOT EXISTS "${RESTOCKER_TEST_RUNNER}")
  message(FATAL_ERROR
    "restocker test isolation runner is missing: ${RESTOCKER_TEST_RUNNER}")
endif()
if(NOT COMMAND add_launch_test)
  message(FATAL_ERROR
    "include restocker_test_isolation.cmake after find_package(launch_testing_ament_cmake)")
endif()

function(add_launch_test filename)
  # RUNNER lands in add_launch_test's UNPARSED_ARGUMENTS and is forwarded to ament_add_test,
  # which invokes it exactly as it would the stock run_test.py.
  _add_launch_test("${filename}" RUNNER "${RESTOCKER_TEST_RUNNER}" ${ARGN})
endfunction()

if(COMMAND ament_add_gtest)
  # A macro, like the upstream one it shadows, so the caller's scope sees the same variables.
  macro(ament_add_gtest target)
    _ament_add_gtest(${target} RUNNER "${RESTOCKER_TEST_RUNNER}" ${ARGN})
  endmacro()
endif()

if(COMMAND ament_add_pytest_test)
  # A function, like the upstream one it shadows.
  function(ament_add_pytest_test testname path)
    _ament_add_pytest_test(
      "${testname}" "${path}" RUNNER "${RESTOCKER_TEST_RUNNER}" ${ARGN})
  endfunction()
endif()

# Runs once this directory's CMakeLists.txt has been processed. A test command that exists
# without its preserved upstream twin was defined after this file was included, so every test
# it added runs unleased on the shell's domain.
function(_restocker_check_test_isolation)
  foreach(command add_launch_test ament_add_gtest ament_add_pytest_test)
    if(COMMAND ${command} AND NOT COMMAND _${command})
      message(FATAL_ERROR
        "${command} is not routed through the ROS domain lease: include "
        "restocker_test_isolation.cmake after the find_package() that defines it")
    endif()
  endforeach()
endfunction()
cmake_language(DEFER CALL _restocker_check_test_isolation)
