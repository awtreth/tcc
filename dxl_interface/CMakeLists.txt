cmake_minimum_required(VERSION 2.8.3)

project(dxl_interface CXX)

add_definitions(-std=c++11 -Wall)

include_directories(include)

set(dxl_interface_srcs
    src/dxl_handle.cpp
    src/dxl_handle_group.cpp
    src/dxl_channel.cpp
    src/dxl_model_spec.cpp
    src/dxl_command.cpp
    src/dxl_control_table.cpp)

set(dxl_interface_headers
    include/dxl_channel.h
    include/dxl_command.h
    include/dxl_control_table.h
    include/dxl_handle.h
    include/dxl_handle_group.h
    include/dxl_interface.h
    include/dxl_model_spec.h)

set(dxl_libs dxl_x64_cpp rt pthread)

add_library(dxl_interface SHARED ${dxl_interface_srcs})
target_link_libraries(dxl_interface ${dxl_libs} yaml-cpp)

add_executable(dxl_interface_test tests/dxl_interface_test.cpp)
target_link_libraries(dxl_interface_test dxl_interface yaml-cpp)

install (TARGETS dxl_interface
    DESTINATION lib)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h")

install(DIRECTORY model_specs/
    DESTINATION share/${PROJECT_NAME}/model_specs
    FILES_MATCHING PATTERN "*.dxl")

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h" modelspecs "*.dxl")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
add_custom_target("${PROJECT_NAME}_modelspecs" SOURCES ${modelspecs})
