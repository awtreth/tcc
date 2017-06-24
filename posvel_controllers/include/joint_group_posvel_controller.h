#ifndef POSVEL_CONTROLLERS_JOINT_GROUP_POSVEL_CONTROLLER_H
#define POSVEL_CONTROLLERS_JOINT_GROUP_POSVEL_CONTROLLER_H

#include <forward_joint_trajectory_point_controller.h>
#include <hardware_interface/posvel_command_interface.h>

namespace posvel_controllers
{


typedef forward_trajectory_point_controller::ForwardJointTrajectoryPointController<hardware_interface::PosVelJointInterface>
        JointGroupPosVelController;


}

#endif
