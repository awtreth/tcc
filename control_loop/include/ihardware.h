#ifndef IHARDWARE_INTERFACE_H
#define IHARDWARE_INTERFACE_H

#include <memory>

using namespace std;

namespace control_loop{


/**
 * @brief Represents a 'Hardware' to ControlLoop. Similar to hardware_interface::RobotHW of ros_control.
 *
 * The way the controllers interact with this entity is up to the user implementation.
 *
 */
class IHardware {

    public:

    /**
     * @brief Send the last specified commands to actuadors
     *
     */
    virtual void write() = 0;

    /**
     * @brief Read values from robot's sensors
     *
     */
    virtual void read() = 0;

    virtual ~IHardware(){}
};

typedef shared_ptr<IHardware> HardwarePtr;

}

#endif
