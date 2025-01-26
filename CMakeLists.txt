# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

project(attitude)

set (CMAKE_CXX_COMPILER g++-14)
set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (BUILD_SHARED_LIBS ON)
if (${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")
  set(TEST_LINK_FLAGS "")
  set(MODULE_OUTPUT_DIR ".")
else()
  set(TEST_LINK_FLAGS LINK_FLAGS "-Wl,--no-as-needed")
  set(MODULE_OUTPUT_DIR "${CMAKE_BUILD_TYPE}")
endif()

find_package(pybind11 REQUIRED)
find_package(Boost 1.80.0 REQUIRED
  system date_time filesystem log log_setup unit_test_framework regex coroutine chrono
  program_options
)
find_package(Threads REQUIRED)
find_package(fmt REQUIRED)

execute_process(COMMAND python-config --includes OUTPUT_VARIABLE Python_INCLUDES)
include_directories(
  ${Boost_INCLUDE_DIR}
  ${fmt_INCLUDE_DIR}
  ${pybind11_INCLUDE_DIR}
)

execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE GITREV)
set(ATTITUDE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(ATTITUDE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/attitudexx")
set(VERSION_H_IN ${ATTITUDE_SOURCE_DIR}/version.h.in)
set(VERSION_H ${ATTITUDE_SOURCE_DIR}/version.h)
set(CONFIG_H_IN ${ATTITUDE_SOURCE_DIR}/config.h.in)
set(CONFIG_H ${ATTITUDE_SOURCE_DIR}/config.h)
configure_file(${VERSION_H_IN} ${VERSION_H} @ONLY)
configure_file(${CONFIG_H_IN} ${CONFIG_H} @ONLY)
if (${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")
configure_file(${CMAKE_SOURCE_DIR}/cmake/CMakeSettings.json.in ${CMAKE_SOURCE_DIR}/CMakeSettings.json @ONLY)
endif()

FILE(GLOB lib_sources src/*.cpp)

add_library(attitude OBJECT ${lib_sources})
set_property(TARGET attitude PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(attitude PRIVATE
  fmt::fmt
)
add_library(attitude_shared SHARED $<TARGET_OBJECTS:attitude>)
add_library(attitude_static STATIC $<TARGET_OBJECTS:attitude>)
set_target_properties(attitude_shared PROPERTIES OUTPUT_NAME attitude)
set_target_properties(attitude_static PROPERTIES OUTPUT_NAME attitude)
pybind11_add_module(attitudexx MODULE ${ATTITUDE_SOURCE_DIR}/attitudexx/binding.cpp ${lib_sources})

enable_testing()

message("Module output directory: ${MODULE_OUTPUT_DIR}")
set_target_properties(attitude PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MODULE_OUTPUT_DIR})

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
    fmt::fmt
    Boost::system
    Boost::filesystem
    Boost::program_options
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::chrono
    Boost::coroutine
    Boost::unit_test_framework
    attitude_static
  )
  add_test(${NEW_TEST} tests/${NEW_TEST})
endmacro()

new_test(test_types)


