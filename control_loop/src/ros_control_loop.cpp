#include <ros_control_loop.h>

control_loop::RosControlLoop::RosControlLoop(std::shared_ptr<hardware_interface::RobotHW> rh, const ros::NodeHandle &nh):
    ControlLoop(std::make_shared<RobotHWAdapter>(rh.get())){

    this->loadController("ControllerManager", std::make_shared<RosControllerManagerAdapter>(rh.get(),nh));
}

void control_loop::RosControlLoop::onMiss(chrono::steady_clock::time_point desire, chrono::steady_clock::time_point realization){
    auto rosNow = ros::Time::now();

    auto timeDiff = std::chrono::duration_cast<std::chrono::nanoseconds>(realization-desire);

    auto rosTimeDiff = ros::Duration(timeDiff.count()/int32_t(1e9),timeDiff.count()%int32_t(1e9));

    onMiss(rosNow - rosTimeDiff,rosNow, rosTimeDiff);
}

void control_loop::RosControlLoop::onMiss(ros::Time desire, ros::Time realization, ros::Duration delay){
    std::cout << "ON MISS (ROS):"
              << "\tDesire=" << (desire.sec + desire.nsec/1e9)
              << "\tRealization=" << (realization.sec + realization.nsec/1e9)
              << "\tDelay="<< delay.sec*1000 + delay.nsec/1e6 << "ms\t"
              << "\tPeriod="<< (1/getFrequency())*1e3 << "ms\t"
              << std::endl;
}
