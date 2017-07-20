#include <iostream>
#include <controller_manager/controller_manager.h>
#include <DxlRobotHW.h>
#include <dynamixel_sdk.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <ros_control_loop.h>
#include <memory>
#include <chrono>
#include <cmath>
#include <utility>

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define RAD_DEGREE_RATIO    (M_PI/180.)


int main(int argc, char** argv){

    //ROS init
    ros::init(argc, argv, "BioloidExampleTest");

    ros::NodeHandle nh;

    //Spinner required for ControllerManager and any Controller that provide ros topics or services
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Joints setup
    std::vector<JointID> jointIDs;

    //joint name, Dynamixel ID, reference, direction
    jointIDs.push_back(JointID("left_shoulder_swing_joint",     1,   60*RAD_DEGREE_RATIO,  -1));
    jointIDs.push_back(JointID("left_shoulder_lateral_joint",   3,  150*RAD_DEGREE_RATIO,  -1));
    jointIDs.push_back(JointID("left_elbow_joint",              5,  150*RAD_DEGREE_RATIO,  -1));

    jointIDs.push_back(JointID("right_shoulder_swing_joint",    6,  240*RAD_DEGREE_RATIO,  +1));
    jointIDs.push_back(JointID("right_shoulder_lateral_joint",  2,  240*RAD_DEGREE_RATIO,  +1));
    jointIDs.push_back(JointID("right_elbow_joint",             4,  150*RAD_DEGREE_RATIO,  +1));

    jointIDs.push_back(JointID("head_pitch_joint",  20,  150*RAD_DEGREE_RATIO,  -1));
    jointIDs.push_back(JointID("head_yaw_joint",    21,  150*RAD_DEGREE_RATIO,  +1));

    //RobotHW initialization
    auto hw = std::make_shared<DxlRobotHW>(jointIDs);

    //ControlLoop + ControllerManager initialization
    control_loop::RosControlLoop ctimer(hw,nh);

    ctimer.setFrequency(50);//in Hz

    ctimer.resumeLoop();

    sleep(99999999);

    return 0;
}
