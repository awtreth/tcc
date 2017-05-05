#include <DummyHardwareInterface.h>
#include <MultiThreadHelper.h>
#include <hardware_interface/robot_hw.h>

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

DummyHardwareInterface::DummyHardwareInterface(){}

bool DummyHardwareInterface::read(){
    defaultRead();
    return true;
}

bool DummyHardwareInterface::write(){
    defaultWrite();
    return true;
}

std::string DummyHardwareInterface::getMsg(){
    return MultiThreadHelper::getSharedParam(mtx,msg);
}

void DummyHardwareInterface::setMsg(const std::string &value){
    MultiThreadHelper::setSharedParam(mtx,msg, value);
}
