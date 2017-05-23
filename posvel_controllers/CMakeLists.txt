cmake_minimum_required(VERSION 2.8.3)

project(posvel_controllers CXX)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    roscpp
    trajectory_msgs
    joint_trajectory_controller
    realtime_tools)

add_definitions(-std=c++11 -Wall)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS hardware_interface controller_interface roscpp joint_trajectory_controller trajectory_msgs roscpp realtime_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(posvel_controllers_srcs
    src/posvel_joint_trajectory_controller.cpp)
    #src/joint_group_posvel_controller.cpp)

add_library(${PROJECT_NAME} ${posvel_controllers_srcs})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
