#include <DxlRobotHW.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

using namespace hardware_interface;

#define WORD_LEN            2

#define GOAL_POS_ADDR       32

#define PRESENT_POS_ADDR    36
#define PRESENT_VEL_ADDR    38
#define PRESENT_EFF_ADDR    40

DxlRobotHW::DxlRobotHW(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, std::map<std::string, int> dxlMap):
    readPacket_(portHandler,packetHandler)
{
    portHandler_ = portHandler;
    packetHandler_ = packetHandler;

    for(auto pair : dxlMap){
        auto jointName = pair.first;

        dxlInfos.push_back(DxlInfo(pair.second));
        DxlInfo& dxl = dxlInfos.back();

        jointStateInterface_.registerHandle(JointStateHandle(jointName,&dxl.pos,&dxl.vel,&dxl.eff));
        positionInterface_.registerHandle(JointHandle(jointStateInterface_.getHandle(jointName),&dxl.posCmd));

        registerInterface(&jointStateInterface_);
        registerInterface(&positionInterface_);

        readPacket_.addParam(uint8_t(pair.second),PRESENT_POS_ADDR,WORD_LEN);
        readPacket_.addParam(uint8_t(pair.second),PRESENT_VEL_ADDR,WORD_LEN);
        readPacket_.addParam(uint8_t(pair.second),PRESENT_EFF_ADDR,WORD_LEN);
    }
}

void DxlRobotHW::write()
{
    dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,WORD_LEN);

    std::cout << "WRITE" << std::endl;

    for(auto dxl : dxlInfos){
        std::cout << "ID: " << dxl.id << " POS: " << dxl.posCmd << std::endl;
        writePacket.addParam(dxl.id,(uint8_t*)(&dxl.posCmd));
    }
    writePacket.txPacket();

}

void DxlRobotHW::read()
{
    readPacket_.txRxPacket();

    std::cout << "READ" << std::endl;

    for(auto dxl : dxlInfos){
        dxl.pos = readPacket_.getData(dxl.id,PRESENT_POS_ADDR,WORD_LEN);
        dxl.vel = readPacket_.getData(dxl.id,PRESENT_VEL_ADDR,WORD_LEN);
        dxl.eff = readPacket_.getData(dxl.id,PRESENT_EFF_ADDR,WORD_LEN);
        std::cout << "POS: " << dxl.pos << " VEL: " << dxl.vel << " EFF: " << dxl.eff << std::endl;
    }
}
