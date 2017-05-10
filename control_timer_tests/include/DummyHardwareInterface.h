#ifndef DUMMY_HARDWARE_INTERFACE_H
#define DUMMY_HARDWARE_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <IHardwareInterface.h>
#include <string>
#include <mutex>
#include <memory>

//class DummyHardwareInterface : public IHardwareInterface, public hardware_interface::RobotHW{

//protected:
//    std::mutex mtx;
//    std::string msg = "";

//    void defaultRead();

//    void defaultWrite();

//    hardware_interface::JointStateInterface js_interface_;

//    double x,y,z;

//public:
//    DummyHardwareInterface();
//    virtual ~DummyHardwareInterface(){}

//    // IHardwareInterface interface

//    virtual bool read();

//    virtual bool write();

//    std::string getMsg();

//    void setMsg(const std::string &value);

//};

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <IHardwareInterface.h>

class DummyHardwareInterface : public hardware_interface::RobotHW, public IHardwareInterface,
        public hardware_interface::HardwareInterface
{
public:
    DummyHardwareInterface();

    void read();
    void write();

    std::mutex mtx;
    std::string msg = "";

    void defaultRead();

    void defaultWrite();

    void setMsg(const std::string &value);
    std::string getMsg();

protected:

private:
    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    std::vector<double> joint_effort_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<std::string> joint_name_;
};


typedef std::shared_ptr<DummyHardwareInterface> DummyHardwareInterfacePtr;


#endif
