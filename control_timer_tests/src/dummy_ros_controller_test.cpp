#include <iostream>
#include <DummyRosController.h>
#include <controller_manager/controller_manager.h>
#include <RosControllerManagerAdapter.h>
#include <ControlTimer.h>
#include <DummyHardwareInterface.h>
#include <ros/ros.h>

using namespace controller_manager;
using namespace std::chrono;

int main(int argc, char** argv){

    auto interface = std::make_shared<DummyHardwareInterface>();

    ros::init(argc, argv, "DummyTest");

    ros::NodeHandle nh;

    auto controllerManager = std::make_shared<ControllerManager>(interface.get(),nh);

    auto rosControllerManagerAdapter = std::make_shared<RosControllerManagerAdapter>(controllerManager);

    for(auto claim : interface->getClaims())
        std::cout << claim << std::endl;

    //std::cout << dummyRosController->isRunning() << std::endl;
    //std::cout << interface-> << std::endl;

    //    std::cout << dummyRosController->getHardwareInterfaceType() << std::endl;
//    std::cout << controllerManager->getControllerNames(std::vector<std::string>()) << std::endl;
//    std::cout << dummyRosController->getHardwareInterfaceType() << std::endl;



//    ControlTimer controlTimer(interface);

//    controlTimer.setPeriod(microseconds(long(500e3)));

//    controlTimer.loadController("DummyController",rosControllerManagerAdapter);

//    sleep(1000);


	return 0;
}
