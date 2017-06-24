cmake_minimum_required(VERSION 2.8.3)

project(ilc_feedforward_controller CXX)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    std_msgs
    joint_state_controller
    realtime_tools
    roscpp
    dxl_robot_hw
    control_timer)

add_definitions(-std=c++11 -Wall)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        controller_interface
        hardware_interface
        std_msgs
        joint_state_controller
        realtime_tools
        roscpp
        dxl_robot_hw
        control_timer
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(single_joint_ilc_controller src/single_joint_ilc_controller.cpp)
target_link_libraries(single_joint_ilc_controller ${catkin_LIBRARIES})

add_executable(single_joint_ilc_controller_test tests/single_joint_ilc_controller_test.cpp)
target_link_libraries(single_joint_ilc_controller_test ${catkin_LIBRARIES} single_joint_ilc_controller)

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
