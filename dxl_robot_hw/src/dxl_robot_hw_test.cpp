#include <iostream>
#include <controller_manager/controller_manager.h>
#include <DxlRobotHW.h>
#include <dynamixel_sdk.h>
#include <ros/ros.h>
#include <map>
#include <string>

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

    mapId["MyJoint"] = 5;

    DxlRobotHW hw(mapId);

    ros::NodeHandle nh;

    controller_manager::ControllerManager cm(&hw,nh);

    ros::Duration period(.01);

    while (ros::ok())
    {
        hw.read();
        cm.update(ros::Time::now(), period);
        hw.write();
        period.sleep();
    }

    return 0;
}
