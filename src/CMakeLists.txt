project(tccMateus)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_definitions(-std=c++11 -Wall -O ${CMAKE_CXX_FLAGS_DEBUG})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
include_directories(core)

add_subdirectory(gz_interface)
add_subdirectory(dxl_interface)
add_subdirectory(core)
add_subdirectory(utils)

FILE(GLOB_RECURSE LibFiles "*.h")
add_custom_target(headers SOURCES ${LibFiles})
