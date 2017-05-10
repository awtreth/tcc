#include <DummyHardwareInterface.h>
#include <MultiThreadHelper.h>
//#include <hardware_interface/robot_hw.h>
//#include <hardware_interface/joint_state_interface.h>

void DummyHardwareInterface::defaultRead(){
    std::cout << "READ" << std::endl;
}

void DummyHardwareInterface::defaultWrite(){

    mtx.lock();
    auto copy = msg;
    msg = "";
    mtx.unlock();

    std::cout << "WRITE: " << copy << std::endl;
}

//DummyHardwareInterface::DummyHardwareInterface(){

//    js_interface_.registerHandle(hardware_interface::JointStateHandle("junta",&x,&y,&z));
//    registerInterface(&js_interface_);

//}

//bool DummyHardwareInterface::read(){
//    defaultRead();
//    return true;
//}

//bool DummyHardwareInterface::write(){
//    defaultWrite();
//    return true;
//}

std::string DummyHardwareInterface::getMsg(){
    return msg;
//    return MultiThreadHelper::getSharedParam(mtx,msg);
}

void DummyHardwareInterface::setMsg(const std::string &value){
    msg = value;
    //    MultiThreadHelper::setSharedParam(&mtx,msg, value);
}

DummyHardwareInterface::DummyHardwareInterface()
{
    using namespace hardware_interface;

    // Initialize raw data
    joint_position_.resize(3);
    joint_velocity_.resize(3);
    joint_effort_.resize(3);
    joint_effort_command_.resize(3);
    joint_velocity_command_.resize(3);
    joint_name_.resize(3);

    joint_name_[0] = "hiDOF_joint1";
    joint_position_[0] = 1.0;
    joint_velocity_[0] = 0.0;
    joint_effort_[0] = 0.1;
    joint_effort_command_[0] = 0.0;
    joint_velocity_command_[0] = 0.0;

    joint_name_[1] = "hiDOF_joint2";
    joint_position_[1] = 1.0;
    joint_velocity_[1] = 0.0;
    joint_effort_[1] = 0.1;
    joint_effort_command_[1] = 0.0;
    joint_velocity_command_[1] = 0.0;

    joint_name_[2] = "hiDOF_joint3";
    joint_position_[2] = 1.0;
    joint_velocity_[2] = 0.0;
    joint_effort_[2] = 0.1;
    joint_effort_command_[2] = 0.0;
    joint_velocity_command_[2] = 0.0;

    // Populate hardware interfaces
    js_interface_.registerHandle(JointStateHandle(joint_name_[0], &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]));
    js_interface_.registerHandle(JointStateHandle(joint_name_[1], &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]));
    js_interface_.registerHandle(JointStateHandle(joint_name_[2], &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]));

    ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_effort_command_[0]));
    ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_effort_command_[1]));
    ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[2]), &joint_effort_command_[2]));

    vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_velocity_command_[0]));
    vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_velocity_command_[1]));
    vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[2]), &joint_velocity_command_[2]));

    registerInterface(&js_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&vj_interface_);
    registerInterface(this);

}


void DummyHardwareInterface::read()
{
    defaultRead();
}

void DummyHardwareInterface::write()
{
    defaultWrite();
}
