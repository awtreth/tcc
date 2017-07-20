#ifndef ROBOT_HW_ADAPTER_H
#define ROBOT_HW_ADAPTER_H

#include <control_loop.h>
#include <ihardware.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace control_loop {

class RobotHWAdapter : public IHardware {

private:
    hardware_interface::RobotHW* robotHW;
    ros::Time lastWriteTime;
    ros::Time lastReadTime;

    // IHardware interface
public:
    void write();

    void read();

    RobotHWAdapter(hardware_interface::RobotHW *rh);

    hardware_interface::RobotHW* getRobotHW();

};

}
#endif
