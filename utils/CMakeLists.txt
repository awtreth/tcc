cmake_minimum_required(VERSION 2.8.3)

project(utils CXX)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED)

set(output_libraries
    async_logger
    multi_thread_helper
    )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${output_libraries}
)

INCLUDE_DIRECTORIES(include)

add_library(async_logger SHARED src/AsyncLogger.cpp)
add_library(multi_thread_helper SHARED src/MultiThreadHelper.cpp)

#add_library(utils SHARED src/AsyncLogger.cpp src/MultiThreadHelper.cpp)


#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})

