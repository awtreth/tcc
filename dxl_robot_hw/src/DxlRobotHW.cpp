#include <DxlRobotHW.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <algorithm>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <thread>

using namespace hardware_interface;

#define WORD_LEN            2

#define GOAL_POS_ADDR       30

#define PRESENT_POS_ADDR    36
#define PRESENT_VEL_ADDR    38
#define PRESENT_EFF_ADDR    40



DxlRobotHW::DxlRobotHW(std::vector<JointID> jointIDs, const char *deviceName, const float protocol, const int baud_rate)
{

//    struct sched_param param;

//    //It can't be greater than 98
//    param.__sched_priority = 98;

//    //TODO: success check
//    std::cout << pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) << std::endl;

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

        uint16_t model_number = 12;

        if(packetHandler_->ping(portHandler_,uint8_t(jointID.id),&model_number)==COMM_SUCCESS){
            DxlInfo dxl(jointID,dxl_interface::ModelSpec::getByNumber(model_number,protocol, DEFAULT_MODEL_SPEC_FOLDER,DEFAULT_MODEL_SPEC_FILE_EXTENSION));
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



void DxlRobotHW::read()
{
    // FIXME: restrito a protocol 1.0

    uint16_t values[3] = {512,0,0};

//    int policy;
//    struct sched_param param;

//    auto tid = std::this_thread::get_id();
//    pthread_getschedparam(pthread_self(),&policy,&param);

//    std::cout << tid << " " << policy << " " << param.sched_priority << std::endl;

    //    std::cout << "READ" << std::endl;

    uint8_t error;

    for(DxlInfo& dxl : dxlInfos){

        auto res = packetHandler_->readTxRx(portHandler_,uint8_t(dxl.jointID.id),PRESENT_POS_ADDR,6,reinterpret_cast<uint8_t*>(&values[0]),&error);

//        packetHandler_->printRxPacketError(error);
//        packetHandler_->printTxRxResult(res);

        dxl.pos = dxl.jointID.getDirection()*(dxl.spec.valueToRadian(values[0])-dxl.jointID.reference);
        dxl.vel = dxl.jointID.getDirection()*dxl.spec.valueToVelocity(values[1]);

        if(values[2] < 1024)
            dxl.eff = values[2]/1024.;
        else
            dxl.eff = (1024 - values[2])/1024.;

//        std::cout << "name: " << dxl.jointID.name << " pos: " << dxl.pos << " vel: " << dxl.vel << " eff: " << dxl.eff;
//        std::cout << " pos_dxl: " << values[0] << " vel_dxl: " << values[1] << " eff_dxl: " << values[2] << std::endl;

    }
}

void DxlRobotHW::write()
{
    //FIXME: restrito a protocolo 1.0

    std::vector<std::string>& posIfaceClaimsRT = *posIfaceClaimBuffer.readFromRT();
    std::vector<std::string>& posVelIfaceClaimsRT = *posVelIfaceClaimBuffer.readFromRT();

    if(posIfaceClaimsRT.size() > 0){
        //        std::cout << "WRITE POSITION:" << std::endl;

        dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,2);

        for(auto joint : posIfaceClaimsRT){
            DxlInfo& dxl = dxlInfos[dxlNameIdxMap[joint]];
            dxl.posCmd_dxl = uint16_t(dxl.spec.radianToValue(dxl.jointID.getDirection()*dxl.posCmd+dxl.jointID.reference));
            writePacket.addParam(uint8_t(dxl.jointID.id),reinterpret_cast<uint8_t*>(&dxl.posCmd_dxl));
            //            std::cout << "name: " << dxl.jointID.name << " posCmd: " << dxl.posCmd << "posCmd_dxl: " << dxl.posCmd_dxl << std::endl;
        }

        writePacket.txPacket();
    }

    if(posVelIfaceClaimsRT.size() > 0){

        //        std::cout << "WRITE POSVEL:" << std::endl;

        dynamixel::GroupSyncWrite writePacket(portHandler_,packetHandler_,GOAL_POS_ADDR,4);

        for(auto joint : posVelIfaceClaimsRT){
            DxlInfo& dxl = dxlInfos[dxlNameIdxMap[joint]];

            dxl.posVelCmd_dxl[0] = uint16_t(dxl.spec.radianToValue(dxl.jointID.getDirection()*dxl.posCmd+dxl.jointID.reference));
            dxl.posVelCmd_dxl[1] = uint16_t(dxl.spec.velocityToValue(dxl.velCmd));

            //dxl.posVelCmd_dxl[1] = (dxl.posVelCmd_dxl[1]==0)?1:dxl.posVelCmd_dxl[1];

            writePacket.addParam(uint8_t(dxl.jointID.id),reinterpret_cast<uint8_t*>(dxl.posVelCmd_dxl));
            //            std::cout << "name: " << dxl.jointID.name << " posCmd: " << dxl.posCmd << " velCmd: " << dxl.velCmd;
            //            std::cout << " posCmd_dxl: " << dxl.posVelCmd_dxl[0] <<  " velCmd_dxl: " << dxl.posVelCmd_dxl[1] << std::endl;
        }

        writePacket.txPacket();
    }
}

void DxlRobotHW::read(const ros::Time&, const ros::Duration&) {
    read();
}

void DxlRobotHW::write(const ros::Time&, const ros::Duration&) {
    write();
}

void DxlRobotHW::doSwitch(const std::list<ControllerInfo> &start_list, const std::list<ControllerInfo> & stop_list)
{

    std::cout << "DO SWITCH" << std::endl;


    std::cout << "Stop List" << std::endl;
    for(auto stopInfo : stop_list){
        for(auto ifaceRes : stopInfo.claimed_resources){

            std::cout << ifaceRes.hardware_interface << ":";

            for(auto res : ifaceRes.resources)
                std::cout << " " << res;

            std::cout << std::endl;

            if(ifaceRes.hardware_interface == "hardware_interface::PositionJointInterface"){
                for(auto res : ifaceRes.resources)
                    posIfaceClaimsNonRT.erase(std::remove(posIfaceClaimsNonRT.begin(),posIfaceClaimsNonRT .end(),res));
            }

            if(ifaceRes.hardware_interface == "hardware_interface::PosVelJointInterface"){
                for(auto res : ifaceRes.resources)
                    posVelIfaceClaimsNonRT.erase(std::remove(posVelIfaceClaimsNonRT.begin(),posVelIfaceClaimsNonRT.end(),res));
            }
        }
    }

    std::cout << "Start List" << std::endl;
    for(auto startInfo : start_list){
        for(auto ifaceRes : startInfo.claimed_resources){

            std::cout << ifaceRes.hardware_interface << ":";

            for(auto res : ifaceRes.resources)
                std::cout << " " << res;

            std::cout << std::endl;

            if(ifaceRes.hardware_interface == "hardware_interface::PositionJointInterface"){
                for(auto res : ifaceRes.resources)
                    posIfaceClaimsNonRT.push_back(res);
            }
            if(ifaceRes.hardware_interface == "hardware_interface::PosVelJointInterface"){
                for(auto res : ifaceRes.resources)
                    posVelIfaceClaimsNonRT.push_back(res);
            }
        }
    }

    std::cout << "posIfaceClaims:";
    for(auto res : posIfaceClaimsNonRT)
        std::cout << " " << res;
    std::cout << std::endl;

    std::cout << "posVelIfaceClaims:";
    for(auto res : posVelIfaceClaimsNonRT)
        std::cout << " " << res;
    std::cout << std::endl;

    posIfaceClaimBuffer.writeFromNonRT(posIfaceClaimsNonRT);
    posVelIfaceClaimBuffer.writeFromNonRT(posVelIfaceClaimsNonRT);
}

