#ifndef DUMMY_HARDWARE_INTERFACE_H
#define DUMMY_HARDWARE_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <ihardware.h>
#include <string>
#include <mutex>
#include <memory>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

class DummyHardwareInterface : public hardware_interface::RobotHW, public control_loop::IHardware,
        public hardware_interface::HardwareInterface
{
public:
    DummyHardwareInterface();

    //control_loop::IHardware methods
    void read() override;
    void write() override;

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
