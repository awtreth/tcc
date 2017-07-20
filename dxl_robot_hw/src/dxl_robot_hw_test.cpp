#include <iostream>
#include <controller_manager/controller_manager.h>
#include <DxlRobotHW.h>
#include <dynamixel_sdk.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <control_loop.h>
#include <ros_control_loop.h> //control_loop package (FIXME: install control_loop)
#include <robot_hw_adapter.h>
#include <ros_controller_manager_adapter.h>
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

    //Joint Configuration
    std::vector<JointID> jointIDs;

    //joint name, Dynamixel ID, reference, direction
    jointIDs.push_back(JointID("joint", 1,  180*RAD_DEGREE_RATIO,  1));

    //RobotHW Initialization
    auto hw = std::make_shared<DxlRobotHW>(jointIDs);

    //ControlLoop + ControllerManager initialization
    control_loop::RosControlLoop ctimer(hw,nh);

    ctimer.setFrequency(320);//in Hz

    ctimer.resumeLoop();

    sleep(99999999);

    return 0;
}
