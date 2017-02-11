project(gzInterface)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_definitions(-std=c++11 -Wall -O ${CMAKE_CXX_FLAGS_DEBUG})

include_directories(include)
add_subdirectory(src)
