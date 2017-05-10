#ifndef ROS_CONTROLLER_MANAGER_ADAPTER
#define ROS_CONTROLLER_MANAGER_ADAPTER

#include <controller_manager/controller_manager.h>
#include <IController.h>
#include <ros/ros.h>
#include <memory>

using namespace controller_manager;
using namespace std;


typedef shared_ptr<ControllerManager> ControllerManagerPtr;

class RosControllerManagerAdapter : public IController{

private:

    ros::Time lastUpdateTime = ros::Time::now();
    ControllerManagerPtr controllerManager;

    // IController interface
public:

    RosControllerManagerAdapter();

    RosControllerManagerAdapter(ControllerManagerPtr _controllerManager);

    void loadControllerManager(ControllerManagerPtr _controllerManager);

    virtual bool prepareRead(chrono::steady_clock::time_point);

    virtual bool update(chrono::steady_clock::time_point);

};

#endif
