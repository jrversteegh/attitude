# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

project(attitude)

set (CMAKE_CXX_STANDARD 23)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_FLAGS "-Wall -Wextra -fmax-errors=1")
set (BUILD_SHARED_LIBS ON)
if(NOT DEFINED ${CMAKE_BUILD_TYPE})
  set (CMAKE_BUILD_TYPE "Release")
endif()
if (${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")
  set(TEST_LINK_FLAGS "")
  set(MODULE_OUTPUT_DIR ".")
else()
  set(TEST_LINK_FLAGS LINK_FLAGS "-Wl,--no-as-needed")
  set(MODULE_OUTPUT_DIR "${CMAKE_BUILD_TYPE}")
endif()

if(BUILD_PYTHON)
  find_package(pybind11 REQUIRED)
  execute_process(COMMAND python-config --includes OUTPUT_VARIABLE Python_INCLUDES)
endif()
if(BUILD_TESTS)
  find_package(Boost 1.80.0 REQUIRED
    system date_time filesystem log log_setup unit_test_framework regex
    program_options
  )
  find_package(benchmark REQUIRED)
endif()

find_package(fmt 11.2.0 REQUIRED)
find_package(trix 0.1.0 REQUIRED)

execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE GITREV)
set(ATTITUDE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(ATTITUDE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests")
set(VERSION_H_IN ${ATTITUDE_INCLUDE_DIR}/attitude/version.h.in)
set(VERSION_H ${ATTITUDE_INCLUDE_DIR}/attitude/version.h)
set(CONFIG_H_IN ${ATTITUDE_INCLUDE_DIR}/attitude/config.h.in)
set(CONFIG_H ${ATTITUDE_INCLUDE_DIR}/attitude/config.h)
configure_file(${VERSION_H_IN} ${VERSION_H} @ONLY)
configure_file(${CONFIG_H_IN} ${CONFIG_H} @ONLY)
if (${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")
configure_file(${CMAKE_SOURCE_DIR}/cmake/CMakeSettings.json.in ${CMAKE_SOURCE_DIR}/CMakeSettings.json @ONLY)
endif()

include_directories(
  ${Boost_INCLUDE_DIR}
  ${pybind11_INCLUDE_DIR}
  ${fmt_INCLUDE_DIR}
  ${trix_INCLUDE_DIR}
  ${ATTITUDE_INCLUDE_DIR}
)
add_compile_definitions(
  FMT_HEADER_ONLY=1
)

FILE(GLOB lib_sources src/*.cpp)

add_library(attitude OBJECT ${lib_sources})
if(BUILD_SHARED)
  set_property(TARGET attitude PROPERTY POSITION_INDEPENDENT_CODE ON)
  add_library(attitude_shared SHARED $<TARGET_OBJECTS:attitude>)
  set_target_properties(attitude_shared PROPERTIES OUTPUT_NAME attitude)
endif()
add_library(attitude_static STATIC $<TARGET_OBJECTS:attitude>)
set_target_properties(attitude_static PROPERTIES OUTPUT_NAME attitude)
if(BUILD_PYTHON)
  pybind11_add_module(attitudexx MODULE ${ATTITUDE_SOURCE_DIR}/attitudexx/binding.cpp ${lib_sources})
endif()

enable_testing()

message("Module output directory: ${MODULE_OUTPUT_DIR}")
set_target_properties(attitude PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MODULE_OUTPUT_DIR})

if(BUILD_TESTS)
  if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
      include(CTest)
  endif()

  macro(new_test NEW_TEST)
    set(TEST_SOURCES ${ARGN})
    list(TRANSFORM TEST_SOURCES PREPEND ${ATTITUDE_SOURCE_DIR}/ )
    add_executable(${NEW_TEST}
      ${TEST_DIR}/${NEW_TEST}.cpp
      ${TEST_SOURCES}
    )
    target_include_directories(${NEW_TEST} PRIVATE
      ${ATTITUDE_INCLUDE_DIR}
      ${ATTITUDE_SOURCE_DIR}
    )
    set_target_properties(${NEW_TEST} PROPERTIES
      ${TEST_LINK_FLAGS}
      RUNTIME_OUTPUT_DIRECTORY tests
    )
    target_link_libraries(${NEW_TEST} PRIVATE
      Boost::system
      Boost::filesystem
      Boost::program_options
      Boost::log
      Boost::log_setup
      Boost::unit_test_framework
      attitude_static
    )
    add_test(${NEW_TEST} tests/${NEW_TEST})
  endmacro()

  macro(new_benchmark NEW_BENCHMARK)
    set(BENCHMARK_SOURCES ${ARGN})
    list(TRANSFORM BENCHMARK_SOURCES PREPEND ${ATTITUDE_SOURCE_DIR}/ )
    add_executable(${NEW_BENCHMARK}
      ${TEST_DIR}/${NEW_BENCHMARK}.cpp
      ${BENCHMARK_SOURCES}
    )
    target_include_directories(${NEW_BENCHMARK} PRIVATE
      ${ATTITUDE_INCLUDE_DIR}
      ${ATTITUDE_SOURCE_DIR}
    )
    set_target_properties(${NEW_BENCHMARK} PROPERTIES
      ${BENCHMARK_LINK_FLAGS}
      RUNTIME_OUTPUT_DIRECTORY benchmarks
    )
    target_link_libraries(${NEW_BENCHMARK} PRIVATE
      benchmark::benchmark_main
      Boost::system
      Boost::filesystem
      Boost::program_options
      Boost::log
      Boost::log_setup
      Boost::unit_test_framework
      attitude_static
    )
    add_test(${NEW_BENCHMARK} benchmarks/${NEW_BENCHMARK})
  endmacro()

  new_test(test_types)
  new_test(test_quaternion)
  new_benchmark(test_performance)
endif()
