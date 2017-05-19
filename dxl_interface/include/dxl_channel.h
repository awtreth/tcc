#ifndef DXL_CHANNEL_H
#define DXL_CHANNEL_H

#include <dynamixel_sdk.h>
#include <dxl_handle.h>
#include <dxl_handle_group.h>
#include <dxl_command.h>
#include <dxl_model_spec.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE   1000000

namespace dynamixel {

class DxlChannel{

public:

    DxlChannel(const char* device = DEFAULT_DEVICE_NAME, const int baud_rate = DEFAULT_BAUD_RATE);

    DxlHandle& getHandle(uint8_t id);
    DxlHandleGroup& getHandleGroup();

    void read(CommandUnit& unit, float protocol);
    void read(CommandUnit& unit);
    void read(ReadCommand& cmd);

    void write(CommandUnit& unit, float protocol);
    void write(CommandUnit& unit);
    void write(WriteCommand& cmd);

    bool scan(uint8_t id, float protocol);
    bool scan(uint8_t id);
    bool scan();

    PacketHandler *getPacketHandler1() const;
    PacketHandler *getPacketHandler2() const;
    PortHandler *getPortHandler() const;

private:

    //CLASS MEMBERS

    PortHandler* portHandler;
    PacketHandler* packetHandler1;
    PacketHandler* packetHandler2;

    DxlHandleGroup handleGroup;


    //HELPER METHODS

    PacketHandler* getPacketHandlerByProtocolNumber(float protocol);

    template<typename ReadGroup>
    void readGroupData(ReadGroup& group,ReadCommand& cmd){
        group.txRxPacket();

        for(CommandUnit& unit : cmd.getCommandUnits()){
            for(int i = 0; i < unit.getLength();i+=4){
                auto currLength = (unit.getLength()-i)%4;
                currLength = (currLength==0)?4:currLength;
                uint32_t res = group.getData(uint8_t(unit.id),uint16_t(unit.address+i),uint16_t(currLength));
                if(unit.getData()!=NULL){
                    std::copy(reinterpret_cast<uint8_t*>(&res), reinterpret_cast<uint8_t*>(&res+currLength), unit.getData()+i);
                }
                handleGroup.get(unit.id).
                        controlTable.set(unit.address+i, currLength, reinterpret_cast<uint8_t*>(&res));
            }
        }
    }


};

}

#endif
