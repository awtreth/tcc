cmake_minimum_required(VERSION 2.8.3)

project(dxl_tests CXX)

add_definitions(-std=c++11 -Wall -O2 -O3)

add_executable(freq_estimator freq_estimator.cpp)
target_link_libraries(freq_estimator dxl_x64_cpp rt pthread)

add_executable(ilc_test ilc_test.cpp)
target_link_libraries(ilc_test dxl_x64_cpp rt pthread)
