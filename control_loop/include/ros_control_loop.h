#ifndef ROS_CONTROL_LOOP_H
#define ROS_CONTROL_LOOP_H

#include <control_loop.h>
#include <ihardware.h>
#include <icontroller.h>
#include <ros/ros.h>
#include <ros_controller_manager_adapter.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <chrono>
#include <robot_hw_adapter.h>
#include <memory>

namespace control_loop {

class RosControlLoop : public ControlLoop {

public:

    RosControlLoop(std::shared_ptr<hardware_interface::RobotHW> rh, const ros::NodeHandle& nh=ros::NodeHandle());

    // ControlLoop interface
protected:

    void onMiss(chrono::steady_clock::time_point desire, chrono::steady_clock::time_point realization);

    void onMiss(ros::Time desire, ros::Time realization, ros::Duration delay);
};

}
#endif
