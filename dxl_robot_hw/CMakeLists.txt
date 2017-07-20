cmake_minimum_required(VERSION 2.8.3)

project(dxl_robot_hw CXX)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager
    control_loop
    realtime_tools)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++11 -Wall)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS hardware_interface realtime_tools control_loop
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dxl_robot_hw SHARED src/DxlRobotHW.cpp)
add_executable(dxl_robot_hw_test src/dxl_robot_hw_test.cpp)

target_link_libraries(dxl_robot_hw ${catkin_LIBRARIES} dxl_x64_cpp rt pthread dxl_interface)
target_link_libraries(dxl_robot_hw_test ${catkin_LIBRARIES} dxl_robot_hw)

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
