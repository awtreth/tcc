project(gzInterface)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -Wall")

set (msgs
  gz_interface_msgs.proto
)

include_directories(${PROTOBUF_INCLUDE_DIRS})
#Para encontrar fonte das mensagens Proto Buff geradas
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#TODO: verificar como trocar diretório dos arquivos gerados do Proto Buff
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gz_interface_msgs SHARED ${PROTO_SRCS})
add_library(gz_dxl_plugin SHARED GzDxlPlugin.cpp)

add_executable(main main.cpp)


target_link_libraries(gz_interface_msgs ${PROTOBUF_LIBRARY})
target_link_libraries(gz_dxl_plugin gz_interface_msgs ${GAZEBO_LIBRARIES})
target_link_libraries(main gz_interface_msgs ${GAZEBO_LIBRARIES})

