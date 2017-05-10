#ifndef ICONTROLLER_INTERFACE_H
#define ICONTROLLER_INTERFACE_H

#include <chrono>
#include <memory>

using namespace std;

class IController {

public:
    virtual bool prepareRead(std::chrono::steady_clock::time_point now) = 0;

    virtual bool update(std::chrono::steady_clock::time_point now) = 0;

    virtual ~IController(){}
};

typedef shared_ptr<IController> ControllerPtr;

#endif
