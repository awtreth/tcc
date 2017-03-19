
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

add_definitions(${GAZEBO_CXX_FLAGS})

set (msgs
  gz_msgs/GzWriteRequest.proto
  gz_msgs/GzReadRequest.proto
  gz_msgs/GzReadResponse.proto
  gz_msgs/DxlInstructionPacket.proto
  gz_msgs/DxlStatusPacket.proto
)

include_directories(${PROTOBUF_INCLUDE_DIRS})

#Para encontrar fonte das mensagens Proto Buff geradas
include_directories(${CMAKE_CURRENT_BINARY_DIR})
#FIXME: verificar como trocar diretório dos arquivos gerados do Proto Buff
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/core/joint_controller)
include_directories(${CMAKE_SOURCE_DIR}/core/joint_command)
include_directories(${CMAKE_SOURCE_DIR}/utils)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gz_interface_msgs SHARED ${PROTO_SRCS})
add_library(gz_joint_controller SHARED src/GzJointController.cpp)
add_library(simple_plugin SHARED src/GzSimplePlugin.cpp)

