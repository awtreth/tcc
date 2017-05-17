#include <iostream>
#include <controller_manager/controller_manager.h>
#include <DxlRobotHW.h>
#include <dynamixel_sdk.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <ControlTimer.h>
#include <RosControllerManagerAdapter.h>
#include <memory>
#include <chrono>

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel


int main(int argc, char** argv){

    ros::init(argc, argv, "DxlRobotHWTest");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::map<std::string,int> mapId;

    mapId["j1"] = 1;
    mapId["j2"] = 3;

    auto hw = std::make_shared<DxlRobotHW>(mapId);

    ros::NodeHandle nh;

    auto cma = std::make_shared<RosControllerManagerAdapter>(hw.get(),nh);
    ControlTimer ctimer(hw);

    ctimer.setFrequency(40);

    ctimer.loadController("MyController",cma);

    ctimer.resumeLoop();

    sleep(99999999);

    return 0;
}
