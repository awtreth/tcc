#ifndef ICONTROLLER_INTERFACE_H
#define ICONTROLLER_INTERFACE_H

#include <chrono>

class IController {

    bool prepareRead(std::chrono::steady_clock::time_point now) = 0;

    bool update(std::chrono::steady_clock::time_point now) = 0;
};

#endif
