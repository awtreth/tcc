
include_directories(
	joint_command
	joint_controller
	joint_motion
	motion_controller
	${CMAKE_SOURCE_DIR}/utils)
#add_subdirectory(joint_command)
#add_subdirectory(joint_controller)
add_subdirectory(joint_motion)
add_subdirectory(motion_controller)
