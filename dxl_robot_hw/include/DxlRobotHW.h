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
        std::string jointName;
        int id;

        double posCmd;
        double pos;
        double eff;
        double vel;

        uint16_t posCmd_dxl;

        DxlInfo(std::string _jointName, int _id){
            jointName = _jointName;
            id = _id;
        }

        DxlInfo(){}
    };

    dynamixel::PortHandler* portHandler_;
    dynamixel::PacketHandler* packetHandler_;

    std::vector<DxlInfo> dxlInfos;

    hardware_interface::JointStateInterface    jointStateInterface_;
    hardware_interface::PositionJointInterface   positionInterface_;

public:
    DxlRobotHW(std::map<std::string, int> dxlMap, const char* deviceName = "/dev/ttyUSB0", const float protocol = 1.0, const int baud_rate = 1000000);


    // IHardwareInterface interface
public:
    void write();
    void read();

    // RobotHW interface
public:
    bool checkForConflict(const std::list<hardware_interface::ControllerInfo> &info) const;
};
