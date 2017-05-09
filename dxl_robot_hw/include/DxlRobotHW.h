#include <dynamixel_sdk.h>
#include <IHardwareInterface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <map>
#include <string>

class DxlRobotHW : public hardware_interface::RobotHW, public IHardwareInterface{

private:

    struct DxlInfo {
        int id;

        double posCmd;
        double pos;
        double eff;
        double vel;

        DxlInfo(int _id){
            id = _id;
        }

        DxlInfo(){}
    };

    dynamixel::PortHandler* portHandler_;
    dynamixel::PacketHandler* packetHandler_;

    std::vector<DxlInfo> dxlInfos;

    dynamixel::GroupBulkRead readPacket_;
//    dynamixel::GroupSyncRead writePacket_;

    hardware_interface::JointStateInterface    jointStateInterface_;
    hardware_interface::PositionJointInterface   positionInterface_;

public:
    DxlRobotHW(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, std::map<std::string,int> dxlMap);


    // IHardwareInterface interface
public:
    void write();
    void read();
};
