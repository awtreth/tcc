#include <ros_control_loop.h>

control_loop::RosControlLoop::RosControlLoop(std::shared_ptr<hardware_interface::RobotHW> rh, const ros::NodeHandle &nh):
    control_loop::ControlLoop(std::make_shared<RobotHWAdapter>(rh.get())){

    this->loadController("ControllerManager", std::make_shared<RosControllerManagerAdapter>(rh.get(),nh));
    cout.setf(ios::fixed);
}

void control_loop::RosControlLoop::onMiss(chrono::steady_clock::time_point desire, chrono::steady_clock::time_point realization){
    auto rosNow = ros::Time::now();

    auto timeDiff = std::chrono::duration_cast<std::chrono::nanoseconds>(realization-desire);

    auto rosTimeDiff = ros::Duration(int(round(timeDiff.count()/1e9)),timeDiff.count()%long(1e9));

    onMiss(rosNow - rosTimeDiff,rosNow, rosTimeDiff);
}

void control_loop::RosControlLoop::onMiss(ros::Time desire, ros::Time realization, ros::Duration delay){
    std::cout << "ON MISS (ROS):"
              << "\tDelay="<< delay.sec*1000 + delay.nsec/1e6 << "ms"
              << "\tPeriod="<< (1/getFrequency())*1e3 << "ms"
              << "\tDesire=" << double(desire.sec + desire.nsec/1e9)
              << "\tRealization=" << double(realization.sec + realization.nsec/1e9)
              << std::endl;
}
