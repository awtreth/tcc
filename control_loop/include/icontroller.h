#ifndef ICONTROLLER_INTERFACE_H
#define ICONTROLLER_INTERFACE_H

#include <chrono>
#include <memory>

using namespace std;

namespace control_loop{

/**
 * @brief Represents a controller to ControlLoop Similar to controller_interface::ControllerInterface of ros_control.
 *
 * The way the controllers interact with the control_loop::IHardware is up to the user.
 */
class IController {

public:


    /**
     * @brief Method called right after IHardware::write(), designed to specify the parameters of IHardware::read() if needed
     *
     * @param now current time
     * @return bool nothing yet
     */
    virtual bool prepareRead(std::chrono::steady_clock::time_point now) = 0;


    /**
     * @brief Similar to ros_control ControllerInterface::update. Designed to get the last values
     * read from IHardware and specify the commands of the next IHardware::write()
     *
     * @param now current time
     * @return bool nothing yet
     */
    virtual bool update(std::chrono::steady_clock::time_point now) = 0;

    virtual ~IController(){}
};

typedef shared_ptr<IController> ControllerPtr;

}

#endif
