#ifndef DUMMY_CONTROLLER_H
#define DUMMY_CONTROLLER_H

#include <IController.h>
#include <DummyControllerBase.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <DummyHardwareInterface.h>

class DummyController : public DummyControllerBase, public IController{

    // IController interface
public:

    virtual ~DummyController();

    DummyController();

    DummyController(DummyHardwareInterfacePtr iface);

    bool prepareRead(std::chrono::steady_clock::time_point);

    bool update(std::chrono::steady_clock::time_point);

};


#endif
