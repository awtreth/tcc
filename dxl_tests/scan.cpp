#include <iostream>
#include <dynamixel_sdk.h>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

// Default setting
#define DXL_ID                          5                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define LED_ADDR    25
#define LED_ON      1
#define LED_OFF     0

#define GOAL_POS_ADDR       30
#define PRESENT_POS_ADDR    36

#define N_ITERATIONS 100

#define N_IDS 4

int main(){

    const double baud_rates[] = {1e6,500e3,400e3,250e3,200e3,115200.0 ,57600,19200,9600};

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press ENTER to terminate...\n");
        getchar();
        return 0;
    }

    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    for(auto baud_rate : baud_rates){
        portHandler->setBaudRate(int(baud_rate));
        std::cout << "baud rate " << baud_rate << std::endl;
        for(uint8_t id = 1; id < 253; id++)
            if(packetHandler->ping(portHandler,id)==COMM_SUCCESS)
                std::cout << int(id) << " found" << std::endl;
    }

    return 0;

}
