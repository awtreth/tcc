#ifndef DXL_CHANNEL_H
#define DXL_CHANNEL_H

#include <dynamixel_sdk.h>
#include "dxl_handle.h"

#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE   1000000

namespace dynamixel {

class DxlChannel{

public:

//    DxlChannel();

    DxlChannel(const char* device = DEFAULT_DEVICE_NAME, const int baud_rate = DEFAULT_BAUD_RATE);

    DxlChannel(PortHandler* _portHandler);

    void scan();

    DxlHandle& getHandle(int id);
    DxlHandle& getHandle(const char* name);

    void read();
    void write();

private:
    PortHandler* portHandler;
    PacketHandler* packetHandler1;
    PacketHandler* packetHandler2;    

};

}

#endif
