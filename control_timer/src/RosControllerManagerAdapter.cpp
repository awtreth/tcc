#include <RosControllerManagerAdapter.h>

RosControllerManagerAdapter::RosControllerManagerAdapter(){}

RosControllerManagerAdapter::RosControllerManagerAdapter(ControllerManagerPtr _controllerManager){
    loadControllerManager(_controllerManager);
}

void RosControllerManagerAdapter::loadControllerManager(ControllerManagerPtr _controllerManager){
//    controllerManager = ControllerManagerPtr(_controllerManager);
    controllerManager = _controllerManager;
}

bool RosControllerManagerAdapter::prepareRead(std::chrono::steady_clock::time_point){return true;}

bool RosControllerManagerAdapter::update(std::chrono::steady_clock::time_point){
    ros::Time now = ros::Time::now();
    ros::Duration duration = now - lastUpdateTime;

    lastUpdateTime = now;

    controllerManager->update(now,duration);

    return true;
}
