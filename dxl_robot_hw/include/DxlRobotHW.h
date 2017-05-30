#include <dynamixel_sdk.h>
#include <IHardwareInterface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <map>
#include <string>
#include <dxl_interface/dxl_model_spec.h>
#include <utility>
#include <realtime_tools/realtime_buffer.h>

class JointID{
public:
    std::string name;
    int id;
    double reference = 0;

    JointID(){}
    JointID(std::string _name, int _id, double _reference, int _direction){
        name = _name;
        id = _id;
        reference = _reference;
        setDirection(_direction);
    }

    int getDirection() const{
        return direction;
    }

    void setDirection(int value){
        direction = (value >= 0)?1:-1;
    }

private:
    int direction = 1;
};


class DxlRobotHW : public hardware_interface::RobotHW, public IHardwareInterface{

private:

    struct DxlInfo {
//        std::string jointName;
//        int id;
//        double posRef = 0;

        JointID jointID;

        dxl_interface::ModelSpec spec;

        double pos;
        double vel;
        double eff;

        double posCmd = 0;
        double velCmd = 1;

        uint16_t posCmd_dxl;
        uint16_t posVelCmd_dxl[2];


        DxlInfo(JointID _jointID){
            jointID = _jointID;
        }


        DxlInfo(JointID _jointID, dxl_interface::ModelSpec _spec){
            jointID = _jointID;
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

    std::vector<std::string> posIfaceClaimsNonRT;
    std::vector<std::string> posVelIfaceClaimsNonRT;

    realtime_tools::RealtimeBuffer<std::vector<std::string> > posIfaceClaimBuffer;
    realtime_tools::RealtimeBuffer<std::vector<std::string> > posVelIfaceClaimBuffer;

public:
    DxlRobotHW(std::vector<JointID> jointIDs, const char* deviceName = "/dev/ttyUSB0", const float protocol = 1.0, const int baud_rate = 1000000);

    // IHardwareInterface interface

public:
    void write();
    void read();


    // RobotHW interface
public:
    void doSwitch(const std::list<hardware_interface::ControllerInfo> & start_list,
                  const std::list<hardware_interface::ControllerInfo> & stop_list);

};
