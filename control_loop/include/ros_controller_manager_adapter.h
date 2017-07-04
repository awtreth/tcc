#ifndef ROS_CONTROLLER_MANAGER_ADAPTER
#define ROS_CONTROLLER_MANAGER_ADAPTER

#include <controller_manager/controller_manager.h>
#include <icontroller.h>
#include <ros/ros.h>
#include <memory>

using namespace controller_manager;
using namespace std;

namespace control_loop{

class RosControllerManagerAdapter : public IController{

private:

    ros::Time lastUpdateTime = ros::Time::now();
    ControllerManager controllerManager;

    // IController interface
public:

    RosControllerManagerAdapter(hardware_interface::RobotHW *robot_hw,
                                const ros::NodeHandle& nh=ros::NodeHandle());

    ControllerManager& getControllerManager();

    virtual bool prepareRead(chrono::steady_clock::time_point);

    virtual bool update(chrono::steady_clock::time_point);

};

}
#endif
