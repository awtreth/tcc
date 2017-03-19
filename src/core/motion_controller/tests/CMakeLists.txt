
add_executable(main main.cpp)
#add_executable(testeDummySynchronizer testeDummySynchronizer.cpp)

#target_link_libraries(gz_interface_msgs ${PROTOBUF_LIBRARY})
#target_link_libraries(gz_simple_plugin gz_interface_msgs ${GAZEBO_LIBRARIES})
#target_link_libraries(main gz_interface_msgs ${GAZEBO_LIBRARIES} dxl_mem_map dxl_model_param_converter jsoncpp gz_joint_controller)
#target_link_libraries(gz_joint_controller ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARY} gz_interface_msgs)

#target_link_libraries(gz_interface ${GAZEBO_LIBRARIES} pthread)
target_link_libraries(main pthread motion_controller jointMotion async_logger)
