#ifndef DUMMY_CONTROLLER_H
#define DUMMY_CONTROLLER_H

#include <icontroller.h>
#include <DummyControllerBase.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <DummyHardwareInterface.h>

class DummyController : public DummyControllerBase, public control_loop::IController{

    // IController interface
public:

//    virtual ~DummyController();

    DummyController();

    DummyController(DummyHardwareInterface *iface);

    bool prepareRead(std::chrono::steady_clock::time_point);

    bool update(std::chrono::steady_clock::time_point);

};


#endif
