#include <iostream>
#include <controller_manager/controller_manager.h>
#include <DxlRobotHW.h>
#include <dynamixel_sdk.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <ros_control_loop.h> //control_loop package (FIXME: install control_loop)
#include <memory>
#include <chrono>
#include <cmath>
#include <utility>

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define RAD_DEGREE_RATIO    M_PI/180.


int main(int argc, char** argv){

    //ROS INIT
    ros::init(argc, argv, "DxlRobotHWTest");

    ros::NodeHandle nh;

    //Spinner required for ControllerManager and any Controller that provide ros topics or services
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //JOINTS CONFIGURATION
    std::vector<JointID> jointIDs;

    jointIDs.push_back(JointID("joint",   3,  150*RAD_DEGREE_RATIO,  -1));

    auto hw = std::make_shared<DxlRobotHW>(jointIDs);

    control_loop::RosControlLoop ctimer(hw,nh);

    ctimer.setFrequency(30);

    ctimer.resumeLoop();

    sleep(99999999);

    return 0;
}
