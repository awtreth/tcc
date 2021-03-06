#include <iostream>
#include <DummyRosController.h>
#include <controller_manager/controller_manager.h>
#include <ros_controller_manager_adapter.h>
#include <control_loop.h>
#include <DummyHardwareInterface.h>
#include <ros/ros.h>

#include <string>
#include <vector>

using namespace controller_manager;
using namespace std::chrono;

int main(int argc, char** argv){


    ros::init(argc, argv, "DummyTest");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto hw = std::make_shared<DummyHardwareInterface>();

    ros::NodeHandle nh;

    auto cma = std::make_shared<control_loop::RosControllerManagerAdapter>(hw.get(),nh);

    control_loop::ControlLoop controlTimer(hw);

    controlTimer.setFrequency(50);

    controlTimer.loadController("MyController",cma);

    controlTimer.resumeLoop();

    sleep(9999999);

    return 0;
}
