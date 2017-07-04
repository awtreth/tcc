cmake_minimum_required(VERSION 2.8.3)

project(bioloid_example CXX)

find_package(catkin REQUIRED
    dxl_robot_hw
    controller_manager
    control_loop)

add_definitions(-std=c++11 -Wall)

catkin_package(CATKIN_DEPENDS dxl_robot_hw controller_manager control_loop)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(bioloid_example src/bioloid_example_main.cpp)
target_link_libraries(bioloid_example ${catkin_LIBRARIES})

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
