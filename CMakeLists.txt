# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (BUILD_SHARED_LIBS ON)

project(attitude)

FILE(GLOB lib_sources src/*.cpp)

add_library(attitude OBJECT ${lib_sources})
set_property(TARGET attitude PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(attitude_shared SHARED $<TARGET_OBJECTS:objlib>)
add_library(attitude_static STATIC $<TARGET_OBJECTS:objlib>)
