#include <iostream>
#include <controller_manager/controller_manager.h>
#include <DxlRobotHW.h>
#include <dynamixel_sdk.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <control_loop.h> //control_loop package (FIXME: install control_loop)
#include <ros_controller_manager_adapter.h>  //control_loop package (FIXME: install control_loop)
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

    ros::init(argc, argv, "DxlRobotHWTest");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::vector<JointID> jointIDs;

    //~ jointIDs.push_back(JointID("left_shoulder_swing_joint",     1,  60*RAD_DEGREE_RATIO,  -1));
    //~ jointIDs.push_back(JointID("left_shoulder_lateral_joint",   3,  150*RAD_DEGREE_RATIO,  -1));
    //~ jointIDs.push_back(JointID("left_elbow_joint",              5,  150*RAD_DEGREE_RATIO,  -1));

    //~ jointIDs.push_back(JointID("right_shoulder_swing_joint",    6,  240*RAD_DEGREE_RATIO,  +1));
    //~ jointIDs.push_back(JointID("right_shoulder_lateral_joint",  2,  240*RAD_DEGREE_RATIO,  +1));
    //~ jointIDs.push_back(JointID("right_elbow_joint",             4,  150*RAD_DEGREE_RATIO,  +1));

    //~ jointIDs.push_back(JointID("head_pitch_joint",  20,  150*RAD_DEGREE_RATIO,  -1));
    //~ jointIDs.push_back(JointID("head_yaw_joint",    21,  150*RAD_DEGREE_RATIO,  +1));

    jointIDs.push_back(JointID("joint",   3,  150*RAD_DEGREE_RATIO,  -1));

    auto hw = std::make_shared<DxlRobotHW>(jointIDs);

    auto cma = std::make_shared<control_loop::RosControllerManagerAdapter>(hw.get(),nh);
    control_loop::ControlLoop ctimer(hw);

    ctimer.setFrequency(30);

    ctimer.loadController("MyController",cma);

    ctimer.resumeLoop();

    sleep(99999999);

    return 0;
}
