cmake_minimum_required(VERSION 2.8.3)

project(control_timer CXX)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager)

add_definitions(-std=c++11 -Wall)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS hardware_interface controller_manager
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(control_timer_srcs
    src/ControlTimer.cpp
    src/RosControllerManagerAdapter.cpp)

add_library(control_timer SHARED ${control_timer_srcs})
target_link_libraries(control_timer ${catkin_LIBRARIES})

#add_executable(ros_control_timer_test src/ros_control_timer_text.cpp)
#set_target_properties(control_timer PROPERTIES LINKER_LANGUAGE CXX)

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
