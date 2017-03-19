include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/utils)
add_library(jointMotion SHARED src/Pose.cpp src/Page.cpp src/PageSet.cpp)

