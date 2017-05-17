cmake_minimum_required(VERSION 2.8.3)

project(dxl_interface CXX)

add_definitions(-std=c++11 -Wall)

include_directories(include)

set(dxl_interface_srcs
    src/dxl_handle.cpp
    src/dxl_channel.cpp
    src/dxl_model_spec.cpp)

set(dxl_libs dxl_x64_cpp rt pthread)

add_library(dxl_interface SHARED ${dxl_interface_srcs})
target_link_libraries(dxl_interface ${dxl_libs} yaml-cpp)

add_executable(dxl_interface_test tests/dxl_interface_test.cpp)
target_link_libraries(dxl_interface_test dxl_interface yaml-cpp)

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
