#ifndef DXL_CHANNEL_H
#define DXL_CHANNEL_H

#include <dynamixel_sdk.h>
#include "dxl_handle.h"
#include "dxl_id.h"

namespace dynamixel {

class DxlChannel{

public:

    DxlChannel();

    DxlChannel(PortHandler* _portHandler);

    DxlChannel(const char* device, const int baud_rate);

    void scan();

    DxlHandle getHandle(DxlId id);

    void write();

    void read();

private:
    PortHandler* portHandler;
    PacketHandler* packetHandler1;
    PacketHandler* packetHandler2;    

};

}

#endif
