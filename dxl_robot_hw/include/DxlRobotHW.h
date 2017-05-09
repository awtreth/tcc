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

        uint16_t posCmd_dxl;
        uint16_t pos_dxl;
        uint16_t eff_dxl;
        uint16_t vel_dxl;

        DxlInfo(int _id){
            id = _id;
        }

        DxlInfo(){}
    };

    dynamixel::PortHandler* portHandler_;
    dynamixel::PacketHandler* packetHandler_;

    std::vector<DxlInfo> dxlInfos;

//    dynamixel::GroupBulkRead readPacket_;
//    dynamixel::GroupSyncRead writePacket_;

    hardware_interface::JointStateInterface    jointStateInterface_;
    hardware_interface::PositionJointInterface   positionInterface_;

public:
    DxlRobotHW(std::map<std::string, int> dxlMap, const char* deviceName = "/dev/ttyUSB0", const float protocol = 1.0, const int baud_rate = 1000000);


    // IHardwareInterface interface
public:
    void write();
    void read();
};
