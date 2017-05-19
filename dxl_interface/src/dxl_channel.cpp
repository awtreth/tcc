#include <dxl_channel.h>

dynamixel::DxlChannel::DxlChannel(const char *device, const int baud_rate){
    portHandler = dynamixel::PortHandler::getPortHandler(device);

    packetHandler1 = dynamixel::PacketHandler::getPacketHandler(1);
    packetHandler2 = dynamixel::PacketHandler::getPacketHandler(2);

    if (portHandler->openPort())
        std::cout << "Succeeded to open the port!" << std::endl;
    else{
        std::cout << "Failed to open the port!" << std::endl;
        return;
    }

    if (portHandler->setBaudRate(baud_rate))
        std::cout << "Succeeded to change the baudrate!" << std::endl;
    else{
        std::cout << "Failed to change the baudrate!" << std::endl;
        return;
    }
}

dynamixel::DxlHandle &dynamixel::DxlChannel::getHandle(uint8_t id){

    if(!handleGroup.hasHandle(id))
        scan(id);

    return handleGroup.get(id);
}

dynamixel::DxlHandleGroup &dynamixel::DxlChannel::getHandleGroup()
{
    return handleGroup;
}


void dynamixel::DxlChannel::read(dynamixel::ReadCommand &cmd){

    switch(cmd.getInstruction()){

    case INST_READ:
        for(CommandUnit& unit : cmd.getCommandUnits())
            read(unit,cmd.getProtocol());
        break;

    case INST_BULK_READ:{
        auto packetHandler = getPacketHandlerByProtocolNumber(cmd.getProtocol());
        dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

        for(CommandUnit& unit : cmd.getCommandUnits())
            groupBulkRead.addParam(uint8_t(unit.id),uint16_t(unit.address), uint16_t(unit.getLength()));

        readGroupData(groupBulkRead,cmd);

        break;
    }
    case INST_SYNC_READ:{

        dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler2,uint16_t(cmd.getFirst().address),uint16_t(cmd.getFirst().getLength()));

        for(CommandUnit& unit : cmd.getCommandUnits())
            groupSyncRead.addParam(uint8_t(unit.id));

        readGroupData(groupSyncRead,cmd);

        break;
    }
    }//switch bracket
}

void dynamixel::DxlChannel::write(dynamixel::WriteCommand &cmd){
    switch(cmd.getInstruction()){

    case INST_WRITE:
        for(CommandUnit& unit : cmd.getCommandUnits())
            write(unit,cmd.getProtocol());
        break;

    case INST_BULK_WRITE:{
        dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler2);

        for(CommandUnit& unit : cmd.getCommandUnits())
            groupBulkWrite.addParam(uint8_t(unit.id),uint16_t(unit.address), uint16_t(unit.getLength()), unit.getData());

        groupBulkWrite.txPacket();

        for(CommandUnit& unit : cmd.getCommandUnits())
            handleGroup.get(unit.id).controlTable.set(unit.address,unit.getLength(),unit.getData());

        break;
    }
    case INST_SYNC_WRITE:{

        auto packetHandler = getPacketHandlerByProtocolNumber(cmd.getProtocol());
        dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler,uint16_t(cmd.getFirst().address),uint16_t(cmd.getFirst().getLength()));

        for(CommandUnit& unit : cmd.getCommandUnits())
            groupSyncWrite.addParam(uint8_t(unit.id),unit.getData());

        groupSyncWrite.txPacket();

        for(CommandUnit& unit : cmd.getCommandUnits())
            handleGroup.get(unit.id).controlTable.set(unit.address,unit.getLength(),unit.getData());

        break;
    }
    }//switch bracket
}

void dynamixel::DxlChannel::read(dynamixel::CommandUnit &unit, float protocol){
    auto packetHandler = getPacketHandlerByProtocolNumber(protocol);

    uint8_t* ptr = (unit.getData()==NULL)?handleGroup.get(unit.id).controlTable.getPtr(unit.address):unit.getData();

    packetHandler->readTxRx(portHandler,uint8_t(unit.id),uint16_t(unit.address),uint16_t(unit.getLength()),ptr);

    //std::cout << int(ptr[0]) << " " << int(ptr[1]) << " " << int(ptr[2]) << " " << int(ptr[3]) << std::endl;

    if(unit.getData()!=NULL){
        handleGroup.get(unit.id).
                controlTable.set(unit.address, unit.getLength(), unit.getData());
    }
}

void dynamixel::DxlChannel::write(dynamixel::CommandUnit &unit, float protocol){
    auto packetHandler = getPacketHandlerByProtocolNumber(protocol);
    packetHandler->writeTxOnly(portHandler,uint8_t(unit.id),uint16_t(unit.address),uint16_t(unit.getLength()),unit.getData());
    handleGroup.get(unit.id).
            controlTable.set(unit.address, unit.getLength(), unit.getData());
}

void dynamixel::DxlChannel::read(dynamixel::CommandUnit &unit)
{
    read(unit,handleGroup.get(unit.id).getProtocol());
}

void dynamixel::DxlChannel::write(dynamixel::CommandUnit &unit)
{
    write(unit,handleGroup.get(unit.id).getProtocol());
}

bool dynamixel::DxlChannel::scan(uint8_t id, float protocol){

    PacketHandler* packetHandler;

    if(int(protocol)==1)
        packetHandler = packetHandler1;
    else
        packetHandler = packetHandler2;

    uint16_t modelNum;

    std::cout << "scan " + std::to_string(id) << std::endl;

    auto res = packetHandler->ping(portHandler,id,&modelNum);

    if(res==COMM_SUCCESS){
        auto spec = ModelSpec::getByNumber(modelNum);

        ControlTable cTable(spec);
        packetHandler->readTxRx(portHandler,id,0,uint16_t(cTable.size()),cTable.getPtr(0));

        handleGroup.addHandle(DxlHandle(id,protocol,cTable));
        return true;
    }else{
        packetHandler->printTxRxResult(res);
        return false;
    }
}

bool dynamixel::DxlChannel::scan(uint8_t id){
    return scan(id,1) || scan(id,2);
}

bool dynamixel::DxlChannel::scan(){
    bool res = false;
    for(uint8_t id = 1; id < 253; id++)
        res = res || scan(id);
    return res;
}

dynamixel::PacketHandler *dynamixel::DxlChannel::getPacketHandlerByProtocolNumber(float protocol){
    if(int(protocol)==1)
        return packetHandler1;
    else if(int(protocol)==2)
        return packetHandler2;
    else
        return NULL;
}
