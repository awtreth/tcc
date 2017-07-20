#include <robot_hw_adapter.h>

void control_loop::RobotHWAdapter::write(){
    auto now = ros::Time::now();
    robotHW->write(now,now-lastWriteTime);
    lastWriteTime = now;
}

void control_loop::RobotHWAdapter::read(){
    auto now = ros::Time::now();
    robotHW->read(now,now-lastReadTime);
    lastReadTime = now;

}

control_loop::RobotHWAdapter::RobotHWAdapter(hardware_interface::RobotHW *rh){
    robotHW = rh;
    lastWriteTime = ros::Time::now();
    lastReadTime = ros::Time::now();
}

hardware_interface::RobotHW *control_loop::RobotHWAdapter::getRobotHW()
{
    return robotHW;
}

