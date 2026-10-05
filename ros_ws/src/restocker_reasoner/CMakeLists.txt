# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_reasoner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(strict_json src/strict_json.cpp)
add_library(restocker_reasoner::strict_json ALIAS strict_json)
target_include_directories(
  strict_json
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(strict_json PRIVATE -Wall -Wextra -Wpedantic)

add_library(recovery_advice src/recovery_advice.cpp)
add_library(restocker_reasoner::recovery_advice ALIAS recovery_advice)
target_include_directories(
  recovery_advice
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(recovery_advice PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(recovery_advice PUBLIC strict_json)

add_library(recovery_advisor_port src/recovery_advisor_port.cpp)
add_library(restocker_reasoner::recovery_advisor_port ALIAS recovery_advisor_port)
target_include_directories(
  recovery_advisor_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(recovery_advisor_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(recovery_advisor_port PUBLIC recovery_advice)

add_library(recovery_audit_log src/recovery_audit_log.cpp)
add_library(restocker_reasoner::recovery_audit_log ALIAS recovery_audit_log)
target_include_directories(
  recovery_audit_log
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(recovery_audit_log PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(recovery_audit_log PUBLIC recovery_advice)

add_library(http_recovery_advisor src/http_recovery_advisor.cpp)
add_library(restocker_reasoner::http_recovery_advisor ALIAS http_recovery_advisor)
target_include_directories(
  http_recovery_advisor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(http_recovery_advisor PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(http_recovery_advisor PUBLIC recovery_advisor_port Threads::Threads)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS
    http_recovery_advisor
    recovery_advice
    recovery_advisor_port
    recovery_audit_log
    strict_json
  EXPORT export_restocker_reasoner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_strict_json test/test_strict_json.cpp)
  target_link_libraries(test_strict_json restocker_reasoner::strict_json)
  target_compile_options(test_strict_json PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_recovery_advice test/test_recovery_advice.cpp)
  target_link_libraries(test_recovery_advice restocker_reasoner::recovery_advice)
  target_compile_options(test_recovery_advice PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_recovery_advisor_port test/test_recovery_advisor_port.cpp)
  target_link_libraries(
    test_recovery_advisor_port restocker_reasoner::recovery_advisor_port)
  target_include_directories(
    test_recovery_advisor_port PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_options(test_recovery_advisor_port PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_recovery_audit_log test/test_recovery_audit_log.cpp)
  target_link_libraries(test_recovery_audit_log restocker_reasoner::recovery_audit_log)
  target_compile_options(test_recovery_audit_log PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_http_recovery_advisor test/test_http_recovery_advisor.cpp TIMEOUT 60)
  target_link_libraries(test_http_recovery_advisor restocker_reasoner::http_recovery_advisor)
  target_compile_options(test_http_recovery_advisor PRIVATE -Wall -Wextra -Wpedantic)

  # Recorded llama.cpp answers to three failure queries, replayed against the live client's
  # validator without a model service.
  ament_add_gtest(
    test_recorded_reasoner_responses test/test_recorded_reasoner_responses.cpp)
  target_link_libraries(
    test_recorded_reasoner_responses restocker_reasoner::recovery_advice)
  target_include_directories(
    test_recorded_reasoner_responses PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_options(
    test_recorded_reasoner_responses PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(
    test_recorded_reasoner_responses
    PRIVATE RESTOCKER_RECORDED_RESPONSES="${CMAKE_CURRENT_SOURCE_DIR}/test/recorded_responses")

  # Diagnostic, not a test: asks a live backend one failure question and prints the answer beside
  # the validator's verdict. Built with the tests, never installed.
  add_executable(reasoner_probe test/reasoner_probe_main.cpp)
  target_link_libraries(reasoner_probe restocker_reasoner::http_recovery_advisor)
  target_include_directories(reasoner_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_options(reasoner_probe PRIVATE -Wall -Wextra -Wpedantic)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_restocker_reasoner HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)
ament_package()
