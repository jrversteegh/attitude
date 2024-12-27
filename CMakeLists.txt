# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

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

project(attitude)

FILE(GLOB lib_sources src/*.cpp)

add_library(attitude OBJECT ${lib_sources})
set_property(TARGET attitude PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(attitude_shared SHARED $<TARGET_OBJECTS:attitude>)
add_library(attitude_static STATIC $<TARGET_OBJECTS:attitude>)

execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE GITREV)
set(PROPER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/attitudexx")
set(VERSION_H_IN ${PROPER_SOURCE_DIR}/version.h.in)
set(VERSION_H ${PROPER_SOURCE_DIR}/version.h)
set(CONFIG_H_IN ${PROPER_SOURCE_DIR}/config.h.in)
set(CONFIG_H ${PROPER_SOURCE_DIR}/config.h)
configure_file(${VERSION_H_IN} ${VERSION_H} @ONLY)
configure_file(${CONFIG_H_IN} ${CONFIG_H} @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/CMakeSettings.json.in ${CMAKE_SOURCE_DIR}/CMakeSettings.json @ONLY)

enable_testing()

message("Module output directory: ${MODULE_OUTPUT_DIR}")
set_target_properties(attitudexx PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MODULE_OUTPUT_DIR})

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

macro(new_test NEW_TEST)
  set(TEST_SOURCES ${ARGN})
  list(TRANSFORM TEST_SOURCES PREPEND ${PROPER_SOURCE_DIR}/ )
  add_executable(${NEW_TEST}
    ${TEST_DIR}/${NEW_TEST}.cpp
    ${TEST_SOURCES}
  )
  target_include_directories(${NEW_TEST} PRIVATE
    ${PROPER_SOURCE_DIR}
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
  )
  add_test(${NEW_TEST} tests/${NEW_TEST})
endmacro()

new_test(test_types)


