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

    //    mlockall(MCL_FUTURE);

    //    const int ids[] = {2,3,4,5};
    //    uint8_t data[] = {0,1,0,1};

    //    struct sched_param param;
    //    param.__sched_priority = 51;

    //    std::cout << pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) << std::endl;

    //    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    //    // Initialize PacketHandler instance
    //    // Set the protocol version
    //    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    //    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    //    // Initialize GroupBulkRead instance
    //    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
    //    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, LED_ADDR, 1);

    //    for(int i = 0; i < N_IDS; i++){
    //        groupSyncWrite.addParam(ids[i],&data[i]);
    //        groupBulkRead.addParam(ids[i],LED_ADDR,1);
    //    }

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

    //    std::chrono::nanoseconds times[N_ITERATIONS];
    //    std::chrono::steady_clock::time_point start;

    //    for (int i = 0; i < N_ITERATIONS; i++){
    //        start = std::chrono::steady_clock::now();
    //        groupBulkRead.txRxPacket();
    //        groupSyncWrite.txPacket();
    //        //packetHandler->read2ByteTxRx(portHandler,DXL_ID,PRESENT_POS_ADDR,&readByte);
    //        //packetHandler->write2ByteTxOnly(portHandler,DXL_ID,GOAL_POS_ADDR,200);
    //        times[i] = std::chrono::steady_clock::now()-start;
    //    }


    //    munlockall();

    //    double sum = 0;

    //    for (int i = 0; i < N_ITERATIONS; i++){
    //        //std::cout << times[i].count()/1e6 << std::endl;
    //        sum += times[i].count()/1e6;
    //    }

    //    std::cout << std::endl << "AVG: " << sum/N_ITERATIONS << std::endl;
    //    return 0;
}
