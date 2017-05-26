cmake_minimum_required(VERSION 2.8.3)

project(bioloid_example CXX)

#~ find_package(catkin REQUIRED)

add_definitions(-std=c++11 -Wall)

#~ catkin_package(
   #~ INCLUDE_DIRS include
   #~ LIBRARIES ${PROJECT_NAME}
   #~ CATKIN_DEPENDS
#~ )

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
