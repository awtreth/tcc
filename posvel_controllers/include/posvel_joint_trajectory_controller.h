#ifndef JOINT_TRAJECTORY_POSVEL_CONTROLLER_H
#define JOINT_TRAJECTORY_POSVEL_CONTROLLER_H

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <hardware_interface/posvel_command_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <posvel_joint_interface_adapter.h>

namespace posvel_controllers {

typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::PosVelJointInterface>
          JointTrajectoryController;

}



#endif
