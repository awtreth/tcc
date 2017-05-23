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

    size_t i = 0;
    for(auto pair : dxlMap){

        uint16_t model_number;

        if(packetHandler_->ping(portHandler_,uint8_t(pair.second),&model_number)==COMM_SUCCESS){
            DxlInfo dxl(pair.first,pair.second,dxl_interface::ModelSpec::getByNumber(model_number));
            dxlInfos.push_back(dxl);
            dxlNameIdxMap[pair.first] = i++;
        }
    }

    read();

    for(DxlInfo& dxl : dxlInfos){
        jointStateInterface_.registerHandle(JointStateHandle(dxl.jointName,&dxl.pos,&dxl.vel,&dxl.eff));
        dxl.posCmd = uint16_t(dxl.spec.radianToValue(dxl.posCmd));
        dxl.velCmd = 1;//non-zero to avoid maximum speed
        positionInterface_.registerHandle(JointHandle(jointStateInterface_.getHandle(dxl.jointName),&dxl.posCmd));
        posVelInterface_.registerHandle(PosVelJointHandle(jointStateInterface_.getHandle(dxl.jointName),&dxl.posCmd,&dxl.velCmd));
    }

    registerInterface(&jointStateInterface_);
    registerInterface(&positionInterface_);
    registerInterface(&posVelInterface_);

}

void DxlRobotHW::write()
{
    //FIXME: restrito a protocolo 1.0

    if(positionInterface_.getClaims().size() > 0){
        std::cout << "WRITE POSITION:" << std::endl;

        dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,2);

        for(auto joint : positionInterface_.getClaims()){
            DxlInfo& dxl = dxlInfos[dxlNameIdxMap[joint]];
            dxl.posCmd_dxl = uint16_t(dxl.spec.radianToValue(dxl.posCmd));
            writePacket.addParam(uint8_t(dxl.id),reinterpret_cast<uint8_t*>(&dxl.posCmd_dxl));
            std::cout << "name: " << dxl.jointName << " posCmd: " << dxl.posCmd << std::endl;
        }

        writePacket.txPacket();
    }

    if(posVelInterface_.getClaims().size() > 0){

        std::cout << "WRITE POSVEL:" << std::endl;

        dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,4);

        for(auto joint : posVelInterface_.getClaims()){
            DxlInfo& dxl = dxlInfos[dxlNameIdxMap[joint]];

            dxl.posVelCmd_dxl[0] = uint16_t(dxl.spec.radianToValue(dxl.posCmd));
            dxl.posVelCmd_dxl[1] = uint16_t(dxl.spec.velocityToValue(dxl.velCmd));

            dxl.posVelCmd_dxl[1] = (dxl.posVelCmd_dxl[1]==0)?1:dxl.posVelCmd_dxl[1];

            writePacket.addParam(uint8_t(dxl.id),reinterpret_cast<uint8_t*>(dxl.posVelCmd_dxl));
            std::cout << "name: " << dxl.jointName << " posCmd: " << dxl.posCmd << " velCmd: " << dxl.velCmd  << std::endl;
        }

        writePacket.txPacket();
    }

}

void DxlRobotHW::read()
{
    // FIXME: restrito a protocol 1.0

    uint16_t values[3];

    std::cout << "READ" << std::endl;

    for(DxlInfo& dxl : dxlInfos){

        packetHandler_->readTxRx(portHandler_,uint8_t(dxl.id),PRESENT_POS_ADDR,6,reinterpret_cast<uint8_t*>(values));

        dxl.pos = dxl.spec.valueToRadian(values[0]);
        dxl.vel = dxl.spec.valueToVelocity(values[1]);
        dxl.eff = values[2]/1024.;

        std::cout << "name: " << dxl.jointName << " pos: " << dxl.pos << " vel: " << dxl.vel << " eff: " << dxl.eff << std::endl;

    }
}
