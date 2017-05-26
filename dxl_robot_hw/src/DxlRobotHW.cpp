#include <DxlRobotHW.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <algorithm>

using namespace hardware_interface;

#define WORD_LEN            2

#define GOAL_POS_ADDR       30

#define PRESENT_POS_ADDR    36
#define PRESENT_VEL_ADDR    38
#define PRESENT_EFF_ADDR    40



DxlRobotHW::DxlRobotHW(std::vector<JointID> jointIDs, const char *deviceName, const float protocol, const int baud_rate)
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
    for(auto jointID : jointIDs){

        uint16_t model_number;

        if(packetHandler_->ping(portHandler_,uint8_t(jointID.id),&model_number)==COMM_SUCCESS){
            DxlInfo dxl(jointID,dxl_interface::ModelSpec::getByNumber(model_number));
            dxlInfos.push_back(dxl);
            dxlNameIdxMap[jointID.name] = i++;
        }
    }

    read();

    for(DxlInfo& dxl : dxlInfos){
        jointStateInterface_.registerHandle(JointStateHandle(dxl.jointID.name,&dxl.pos,&dxl.vel,&dxl.eff));
        dxl.posCmd = dxl.pos;
        dxl.velCmd = 0;
        positionInterface_.registerHandle(JointHandle(jointStateInterface_.getHandle(dxl.jointID.name),&dxl.posCmd));
        posVelInterface_.registerHandle(PosVelJointHandle(jointStateInterface_.getHandle(dxl.jointID.name),&dxl.posCmd,&dxl.velCmd));
    }

    registerInterface(&jointStateInterface_);
    registerInterface(&positionInterface_);
    registerInterface(&posVelInterface_);

}

void DxlRobotHW::write()
{
    //FIXME: restrito a protocolo 1.0

    std::vector<std::string> & posIfaceClaimsRT = *posIfaceClaimBuffer.readFromRT();

    if(posIfaceClaimsRT.size() > 0){
//        std::cout << "WRITE POSITION:" << std::endl;

        dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,2);

        for(auto joint : posIfaceClaimsRT){
            DxlInfo& dxl = dxlInfos[dxlNameIdxMap[joint]];
            dxl.posCmd_dxl = uint16_t(dxl.spec.radianToValue(dxl.jointID.getDirection()*dxl.posCmd+dxl.jointID.reference));
            writePacket.addParam(uint8_t(dxl.jointID.id),reinterpret_cast<uint8_t*>(&dxl.posCmd_dxl));
//            std::cout << "name: " << dxl.jointID.name << " posCmd: " << dxl.posCmd << std::endl;
        }

        writePacket.txPacket();
    }

    if(posVelInterface_.getClaims().size() > 0){

//        std::cout << "WRITE POSVEL:" << std::endl;

        dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,4);

        for(auto joint : posVelInterface_.getClaims()){
            DxlInfo& dxl = dxlInfos[dxlNameIdxMap[joint]];

            dxl.posVelCmd_dxl[0] = uint16_t(dxl.spec.radianToValue(dxl.jointID.getDirection()*dxl.posCmd+dxl.jointID.reference));
            dxl.posVelCmd_dxl[1] = uint16_t(dxl.spec.velocityToValue(dxl.velCmd));

            dxl.posVelCmd_dxl[1] = (dxl.posVelCmd_dxl[1]==0)?1:dxl.posVelCmd_dxl[1];

            writePacket.addParam(uint8_t(dxl.jointID.id),reinterpret_cast<uint8_t*>(dxl.posVelCmd_dxl));
//            std::cout << "name: " << dxl.jointID.name << " posCmd: " << dxl.posCmd << " velCmd: " << dxl.velCmd  << std::endl;
        }

        writePacket.txPacket();
    }

}

void DxlRobotHW::read()
{
    // FIXME: restrito a protocol 1.0

    uint16_t values[3];

//    std::cout << "READ" << std::endl;

    for(DxlInfo& dxl : dxlInfos){

        packetHandler_->readTxRx(portHandler_,uint8_t(dxl.jointID.id),PRESENT_POS_ADDR,6,reinterpret_cast<uint8_t*>(values));

        dxl.pos = dxl.jointID.getDirection()*(dxl.spec.valueToRadian(values[0])-dxl.jointID.reference);
        dxl.vel = dxl.spec.valueToVelocity(values[1]);

        if(values[2] < 1024)
            dxl.eff = values[2]/1024.;
        else
            dxl.eff = (1024 - values[2])/1024.;

//        std::cout << "name: " << dxl.jointID.name << " pos: " << dxl.pos << " vel: " << dxl.vel << " eff: " << dxl.eff << std::endl;

    }
}

void DxlRobotHW::doSwitch(const std::list<ControllerInfo> &start_list, const std::list<ControllerInfo> & stop_list)
{

    for(auto stopInfo : stop_list){
        for(auto ifaceRes : stopInfo.claimed_resources){
            if(ifaceRes.hardware_interface == "hardware_interface::PositionJointInterface"){
                for(auto res : ifaceRes.resources)
                    std::remove(posIfaceCaimsNonRT .begin(),posIfaceCaimsNonRT .end(),res);
            }
        }
    }

    for(auto startInfo : start_list){
        for(auto ifaceRes : startInfo.claimed_resources){
            if(ifaceRes.hardware_interface == "hardware_interface::PositionJointInterface"){
                for(auto res : ifaceRes.resources)
                    posIfaceCaimsNonRT.push_back(res);
            }
        }
    }

    posIfaceClaimBuffer.writeFromNonRT(posIfaceCaimsNonRT);
}
