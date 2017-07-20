cmake_minimum_required(VERSION 2.8.3)

project(control_loop CXX)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager)

set(CMAKE_BUILD_TYPE Debug)

add_definitions(-std=c++11 -Wall)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS hardware_interface controller_manager
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(control_loop_srcs
    src/control_loop.cpp
    src/ros_controller_manager_adapter.cpp
    src/robot_hw_adapter.cpp
    src/ros_control_loop.cpp)

add_library(${PROJECT_NAME} SHARED ${control_loop_srcs})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pthread cap)

#Only to show the headers on Qt Creator's Project Panel
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
