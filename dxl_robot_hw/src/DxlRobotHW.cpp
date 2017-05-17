#include <DxlRobotHW.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

using namespace hardware_interface;

#define WORD_LEN            2

#define GOAL_POS_ADDR       30

#define PRESENT_POS_ADDR    36
#define PRESENT_VEL_ADDR    38
#define PRESENT_EFF_ADDR    40

DxlRobotHW::DxlRobotHW(std::map<std::string, int> dxlMap, const char* deviceName, const float protocol, const int baud_rate)
    //:readPacket_(portHandler,packetHandler)
{

    portHandler_ = dynamixel::PortHandler::getPortHandler(deviceName);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol);

    // Open port
    if (portHandler_->openPort())
        printf("Succeeded to open the port!\n");
    else
        printf("Failed to open the port!\n");

    // Set port baudrate
    if (portHandler_->setBaudRate(baud_rate))
        printf("Succeeded to change the baudrate!\n");
    else
        printf("Failed to change the baudrate!\n");

    for(auto pair : dxlMap)
        dxlInfos.push_back(DxlInfo(pair.first,pair.second));

    for(DxlInfo& dxl : dxlInfos){
        dxl.posCmd = 500;
        jointStateInterface_.registerHandle(JointStateHandle(dxl.jointName,&dxl.pos,&dxl.vel,&dxl.eff));
        positionInterface_.registerHandle(JointHandle(jointStateInterface_.getHandle(dxl.jointName),&dxl.posCmd));
    }

    registerInterface(&jointStateInterface_);
    registerInterface(&positionInterface_);

//    read();

//    for(auto : dxlInfos)
//        dxl.posCmd = dxl.pos;
}

void DxlRobotHW::write()
{
    dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,WORD_LEN);

//    std::cout << "WRITE" << std::endl;

    for(DxlInfo& dxl : dxlInfos){
//        std::cout << "ID: " << dxl.id << " POS: " << dxl.posCmd << std::endl;
        dxl.posCmd_dxl = uint16_t(dxl.posCmd);
        writePacket.addParam(dxl.id,(uint8_t*)(&dxl.posCmd_dxl));
    }

    writePacket.txPacket();

}

void DxlRobotHW::read()
{
//    std::cout << "READ" << std::endl;

    uint16_t values[3];

    for(DxlInfo& dxl : dxlInfos){
        auto result = packetHandler_->readTxRx(portHandler_,dxl.id,PRESENT_POS_ADDR,6,(uint8_t*)values);

        dxl.pos = double(values[0]);
        dxl.vel = double(values[1]);
        dxl.eff = double(values[2]);
//        std::cout << "POS: " << dxl.pos << " VEL: " << dxl.vel << " EFF: " << dxl.eff << std::endl;
    }
}

bool DxlRobotHW::checkForConflict(const std::list<ControllerInfo> &info) const
{
    return false;
}
