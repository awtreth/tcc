cmake_minimum_required(VERSION 2.8.3)

project(control_timer_tests CXX)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager
    control_loop
    roscpp
    controller_interface)

add_definitions(-std=c++11 -Wall)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES dummy_controller
   CATKIN_DEPENDS hardware_interface controller_manager control_loop roscpp controller_interface
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(dummy_controller_srcs
    src/DummyControllerBase.cpp
    src/DummyHardwareInterface.cpp
    src/DummyController.cpp
    src/DummyRosController.cpp)

add_library(dummy_controller ${dummy_controller_srcs})
add_executable(dummy_controller_test src/dummy_controller_test.cpp)
add_executable(dummy_ros_controller_test src/dummy_ros_controller_test.cpp)

target_link_libraries(dummy_controller ${catkin_LIBRARIES})
target_link_libraries(dummy_controller_test dummy_controller ${catkin_LIBRARIES})
target_link_libraries(dummy_ros_controller_test ${catkin_LIBRARIES} dummy_controller)


#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
