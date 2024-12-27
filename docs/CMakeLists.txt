cmake_minimum_required(VERSION 3.14)

project(ProperDocs)

if (NOT DEFINED VERSION)
  message(FATAL_ERROR "Definition of VERSION is required.")
endif()

find_package(Doxygen)

set(PROPER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src/properxx")

if(DOXYGEN_FOUND)
  set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
  set(DOXYFILE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  configure_file(${DOXYFILE_IN} ${DOXYFILE} @ONLY)

  add_custom_target(
    doc ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
else()
  add_custom_target(
    doc ALL
    COMMAND echo "Doxygen not installed. Not building C++ documentation."
    COMMENT "Doxygen not installed"
    VERBATIM
  )
endif()
