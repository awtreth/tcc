#include <dynamixel_sdk.h>
#include <IHardwareInterface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <map>
#include <string>
#include <dxl_interface/dxl_model_spec.h>

class DxlRobotHW : public hardware_interface::RobotHW, public IHardwareInterface{

private:

    struct DxlInfo {
        std::string jointName;
        int id;
        dxl_interface::ModelSpec spec;

        double pos;
        double vel;
        double eff;

        double posCmd;
        double velCmd;

        uint16_t posCmd_dxl;
        uint16_t posVelCmd_dxl[2];

        DxlInfo(std::string _jointName, int _id){
            jointName = _jointName;
            id = _id;
        }


        DxlInfo(std::string _jointName, int _id, dxl_interface::ModelSpec _spec){
            jointName = _jointName;
            id = _id;
            spec = _spec;
        }

        DxlInfo(){}
    };

    dynamixel::PortHandler* portHandler_;
    dynamixel::PacketHandler* packetHandler_;

    std::vector<DxlInfo> dxlInfos;
    std::map<std::string,size_t> dxlNameIdxMap;

    hardware_interface::JointStateInterface    jointStateInterface_;
    hardware_interface::PositionJointInterface   positionInterface_;
    hardware_interface::PosVelJointInterface   posVelInterface_;


public:
    DxlRobotHW(std::map<std::string, int> dxlMap, const char* deviceName = "/dev/ttyUSB0", const float protocol = 1.0, const int baud_rate = 1000000);

    // IHardwareInterface interface

public:
    void write();
    void read();

};
