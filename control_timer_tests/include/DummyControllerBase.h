#ifndef DUMMY_CONTROLLER_BASE_H
#define DUMMY_CONTROLLER_BASE_H

#include <chrono>
#include <mutex>
#include <DummyHardwareInterface.h>

class DummyControllerBase{

private:

    std::chrono::microseconds perLetterDuration = std::chrono::microseconds(long(10e3));

    std::string msg = "DUMMY_MESSAGE";

    std::mutex msgMtx;
    std::mutex durationMtx;

protected:
    DummyHardwareInterfacePtr hardwareInterface;

public:

    DummyControllerBase();

    DummyControllerBase(DummyHardwareInterfacePtr iface);

    virtual ~DummyControllerBase();

    void loadInterface(DummyHardwareInterfacePtr iface);

    std::chrono::microseconds getPerLetterDuration();

    void setPerLetterDuration(const std::chrono::microseconds &value);

    std::string getMsg();

    void setMsg(const std::string &value);

    std::chrono::microseconds getTotalTime();

};


#endif
