#include <ros_controller_manager_adapter.h>

using namespace control_loop;

RosControllerManagerAdapter::RosControllerManagerAdapter(hardware_interface::RobotHW *robot_hw, const ros::NodeHandle &nh)
    : controllerManager(robot_hw, nh) {

}

ControllerManager &RosControllerManagerAdapter::getControllerManager()
{
    return controllerManager;
}

bool RosControllerManagerAdapter::prepareRead(std::chrono::steady_clock::time_point){return true;}

bool RosControllerManagerAdapter::update(std::chrono::steady_clock::time_point){
    ros::Time now = ros::Time::now();
    ros::Duration duration = now - lastUpdateTime;

    lastUpdateTime = now;

    controllerManager.update(now,duration);

    return true;
}
