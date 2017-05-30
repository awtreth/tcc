#include <iostream>
#include <dynamixel_sdk.h>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <string>

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

#define N_ITERATIONS 4000

//#define N_IDS 2

//#define BULK_READ

int main(int argc, char** argv){

    int N_IDS = std::stoi(argv[1]);
    int n_bytes = std::stoi(argv[2]);


    struct sched_param param;
    param.__sched_priority = 98;

    bool realtime = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;

    if(realtime)
        mlockall(MCL_FUTURE);

    const uint8_t ids[] = {27,29,31,35,38,56};
    uint8_t data[] = {1,0,1,0,1,0};
    uint8_t read_buff[20];

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupBulkRead instance
#ifdef BULK_READ
    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
#endif
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, LED_ADDR, 1);

    for(int i = 0; i < N_IDS; i++){
        groupSyncWrite.addParam(ids[i],&data[i]);
#ifdef BULK_READ
        groupBulkRead.addParam(ids[i],LED_ADDR,1);
#endif
    }

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

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press ENTER to terminate...\n");
        getchar();
        return 0;
    }

    std::chrono::nanoseconds times[N_ITERATIONS];
    int readResults[N_ITERATIONS];
    std::chrono::steady_clock::time_point start;


    for (int i = 0; i < N_ITERATIONS; i++){
        start = std::chrono::steady_clock::now();
#ifdef BULK_READ
        groupBulkRead.txRxPacket();
//        for(int j = 0; j < N_IDS; j++)
//            std::cout << int(groupBulkRead.getData(ids[j],LED_ADDR,1)) << std::endl;
#else
        for(size_t i = 0; i < N_IDS;i++){
            readResults[i] = packetHandler->read1ByteTxRx(portHandler,ids[i],LED_ADDR,&read_buff[i]);
        }
        //            readResults[i] = packetHandler->readTxRx(portHandler,ids[i],LED_ADDR,uint16_t(n_bytes),read_buff);
#endif
//        groupSyncWrite.txPacket();

        for(size_t i = 0; i < N_IDS;i++){
            packetHandler->write1ByteTxOnly(portHandler,ids[i],LED_ADDR,data[i]);
        }

        times[i] = std::chrono::steady_clock::now()-start;
    }


    if(realtime)
        munlockall();

    double sum = 0;

    std::cout << "iteration, value, result" << std::endl;
    for (int i = 0; i < N_ITERATIONS; i++){
        std::cout << i<< "," << times[i].count()/1e6 << "," << readResults[i] << std::endl;
        sum += times[i].count()/1e6;
    }

    std::cout << std::endl << "AVG," << sum/N_ITERATIONS << std::endl;
    return 0;
}
