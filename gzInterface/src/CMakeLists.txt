
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

add_definitions(${GAZEBO_CXX_FLAGS})

set (msgs
  ${CMAKE_SOURCE_DIR}/gz_msgs/GzWriteRequest.proto
  ${CMAKE_SOURCE_DIR}/gz_msgs/GzReadRequest.proto
  ${CMAKE_SOURCE_DIR}/gz_msgs/GzReadAnswer.proto
  ${CMAKE_SOURCE_DIR}/gz_msgs/DxlInstructionPacket.proto
  ${CMAKE_SOURCE_DIR}/gz_msgs/DxlStatusPacket.proto


)

include_directories(${PROTOBUF_INCLUDE_DIRS})

#Para encontrar fonte das mensagens Proto Buff geradas
include_directories(${CMAKE_CURRENT_BINARY_DIR})
#TODO: verificar como trocar diretório dos arquivos gerados do Proto Buff
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gz_interface_msgs SHARED ${PROTO_SRCS})
add_library(gz_simple_plugin SHARED GzSimplePlugin.cpp)
add_library(dxl_mem_map SHARED DxlMemMap.cpp)
add_library(dxl_model_param_converter SHARED DxlModelParamConverter.cpp)
add_library(funcmov_gz SHARED funcmovgz.cpp)

add_executable(main main.cpp)

target_link_libraries(gz_interface_msgs ${PROTOBUF_LIBRARY})
target_link_libraries(gz_simple_plugin gz_interface_msgs ${GAZEBO_LIBRARIES})
target_link_libraries(main gz_interface_msgs ${GAZEBO_LIBRARIES} dxl_mem_map dxl_model_param_converter jsoncpp)
