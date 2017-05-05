#include <iostream>
#include <ControlTimer.h>
#include <DummyController.h>
#include <DummyHardwareInterface.h>
#include <stdio.h>
#include <ros/ros.h>
#include <memory>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace std::chrono;

int main(int argc, char** argv){

    auto interface = std::make_shared<DummyHardwareInterface>();

//    ros::init(argc, argv, "DummyTest");

//    ros::NodeHandle nh;

//    bool init_success = interface.init(nh, nh);

    auto controller = std::make_shared<DummyController>(interface);

    controller->setMsg("MENSAGEM MAIOR");
    controller->setPerLetterDuration(microseconds(long(100e3)));

    ControlTimer controlTimer(interface);

    controlTimer.setPeriod(microseconds(long(500e3)));

    controlTimer.loadController("DummyController",controller);

    sleep(1000);

	return 0;
}
