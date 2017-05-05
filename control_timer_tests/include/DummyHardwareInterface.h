#ifndef DUMMY_HARDWARE_INTERFACE_H
#define DUMMY_HARDWARE_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <IHardwareInterface.h>
#include <string>
#include <mutex>
#include <memory>

class DummyHardwareInterface : public IHardwareInterface,
        public hardware_interface::HardwareInterface, public hardware_interface::RobotHW{

protected:
    std::mutex mtx;
    std::string msg = "";

    void defaultRead();

    void defaultWrite();

public:
    DummyHardwareInterface();
    virtual ~DummyHardwareInterface(){}

    // IHardwareInterface interface

    virtual bool read();

    virtual bool write();

    std::string getMsg();

    void setMsg(const std::string &value);

};

typedef std::shared_ptr<DummyHardwareInterface> DummyHardwareInterfacePtr;


#endif
