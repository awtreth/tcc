#ifndef ICONTROLLER_INTERFACE_H
#define ICONTROLLER_INTERFACE_H

#include <chrono>

class IController {

public:
    virtual bool prepareRead(std::chrono::steady_clock::time_point now) = 0;

    virtual bool update(std::chrono::steady_clock::time_point now) = 0;
};

#endif
