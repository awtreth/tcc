#include <DummyControllerBase.h>
#include <MultiThreadHelper.h>
#include <DummyHardwareInterface.h>


DummyControllerBase::DummyControllerBase(){}

DummyControllerBase::DummyControllerBase(DummyHardwareInterfacePtr iface){
    loadInterface(iface);
}

DummyControllerBase::~DummyControllerBase(){}

void DummyControllerBase::loadInterface(DummyHardwareInterfacePtr iface)
{
    hardwareInterface = iface;
    //hardwareInterface = DummyHardwareInterfacePtr(iface);
}

std::chrono::microseconds DummyControllerBase::getPerLetterDuration(){
    return MultiThreadHelper::getSharedParam(durationMtx,perLetterDuration);
}

void DummyControllerBase::setPerLetterDuration(const std::chrono::microseconds &value){
    MultiThreadHelper::setSharedParam(durationMtx,perLetterDuration, value);
}

void DummyControllerBase::setMsg(const std::string &value){
    MultiThreadHelper::setSharedParam(msgMtx,msg, value);
    //hardwareInterface->setMsg(value);
}

std::chrono::microseconds DummyControllerBase::getTotalTime()
{
    return perLetterDuration*msg.size();
}

std::string DummyControllerBase::getMsg(){
    return MultiThreadHelper::getSharedParam(msgMtx,msg);
}
