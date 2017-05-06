#include <iostream>
#include <DummyRosController.h>
#include <controller_manager/controller_manager.h>
#include <RosControllerManagerAdapter.h>
#include <ControlTimer.h>
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

    DummyHardwareInterface hw;

    hw.setMsg("message");
    std::cout << hw.getMsg() << std::endl;
    ros::NodeHandle nh;
    controller_manager::ControllerManager cm(&hw, nh);

//    cm.loadController("MyController");
//    std::vector<std::string> controllerList;
//    std::vector<std::string> outputList;
//    controllerList.push_back("MyController");

//    ros::Duration period(1.0);

//    getchar();
//    std::cout << "FOI" << std::endl;

//    cm.switchController(controllerList,outputList,2);
    ros::Duration period(1.0);

    while (ros::ok())
    {
        hw.read();
        cm.update(ros::Time::now(), period);
        hw.write();
        period.sleep();
    }

    //    ros::init(argc, argv, "DummyTest");

    //    ros::NodeHandle nh;

    //    ros::AsyncSpinner spinner(1);
    //    spinner.start();

    ////    auto interface = std::make_shared<DummyHardwareInterface>();
    //    DummyHardwareInterface iface;

    //    ControllerManager controllerManager(&iface,nh);

    //    controllerManager->loadController("DummyRosController");

    //    auto controllerManager = std::make_shared<ControllerManager>(interface.get(),nh);

    //    auto rosControllerManagerAdapter = std::make_shared<RosControllerManagerAdapter>(controllerManager);

    //    ControlTimer controlTimer;

    //    controlTimer.setFrequency(1);

    //    controlTimer.setHardwareInterface(interface);

    //    controllerManager->loadController("DummyRosController");

    //    std::vector<std::string> controllerList;
    //    controllerList.push_back("DummyRosController");

    //    controllerManager->switchController(controllerList,std::vector<std::string>(),2);


    //controlTimer.resumeLoop();

    sleep(9999999);

    return 0;
}
