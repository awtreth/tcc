#include <iostream>
#include <controller_manager/controller_manager.h>
#include <DxlRobotHW.h>
#include <dynamixel_sdk.h>
#include <ros/ros.h>
#include <map>
#include <string>

// Default setting
#define DXL_ID                          5                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel


int main(int argc, char** argv){

    struct sched_param param;
    param.__sched_priority = 51;

    std::cout << pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) << std::endl;

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//    // Open port
//    if (portHandler->openPort())
//    {
//        printf("Succeeded to open the port!\n");
//    }
//    else
//    {
//        printf("Failed to open the port!\n");
//        printf("Press ENTER to terminate...\n");
//        getchar();
//        return 0;
//    }

//    // Set port baudrate
//    if (portHandler->setBaudRate(BAUDRATE))
//    {
//        printf("Succeeded to change the baudrate!\n");
//    }
//    else
//    {
//        printf("Failed to change the baudrate!\n");
//        printf("Press ENTER to terminate...\n");
//        getchar();
//        return 0;
//    }


    ros::init(argc, argv, "DxlRobotHWTest");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::map<std::string,int> mapId;

    mapId["MyJoint"] = 5;

    DxlRobotHW hw(portHandler,packetHandler, mapId);

    ros::NodeHandle nh;

    controller_manager::ControllerManager cm(&hw,nh);

    ros::Duration period(1.0);

    while (ros::ok())
    {
        hw.read();
        cm.update(ros::Time::now(), period);
        hw.write();
        period.sleep();
    }

    return 0;
}
