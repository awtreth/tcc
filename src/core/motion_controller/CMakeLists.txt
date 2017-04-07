
add_subdirectory(tests)
add_library(motion_controller src/MotionController.cpp)
#add_library(posvel_motion_controller SHARED src/PosVelMotionController.cpp)
target_link_libraries(motion_controller jointMotion)
#target_link_libraries(posvel_motion_controller motion_controller)
#add_library(dummy_motion_controller SHARED src/)
