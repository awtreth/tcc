#ifndef FORWARD_TRAJECTORY_POINT_CONTROLLER_FORWARD_JOINT_GROUP_TRAJECTORY_POINT_CONTROLLER_H
#define FORWARD_TRAJECTORY_POINT_CONTROLLER_FORWARD_JOINT_GROUP_TRAJECTORY_POINT_CONTROLLER_H

#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace forward_trajectory_point_controller
{

template<typename T, typename HandleType = typename T::ResourceHandleType>
class ForwardJointTrajectoryPointController: public controller_interface::Controller<T>
{
public:
    ForwardJointTrajectoryPointController() {}
    ~ForwardJointTrajectoryPointController() {sub_command_.shutdown();}

    struct PosVelAccEff{
        double position = 0;
        double velocity = 0;
        double acceleration = 0;
        double effort = 0;

        PosVelAccEff(){}

        PosVelAccEff(double pos, double vel, double acc, double eff):position(pos),velocity(vel),acceleration(acc),effort(eff){}
    };

    bool init(T* hw, ros::NodeHandle &n)
    {
        // List of controlled joints
        std::string param_name = "joints";
        if(!n.getParam(param_name, joint_names_))
        {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
            return false;
        }
        n_joints_ = joint_names_.size();

        if(n_joints_ == 0){
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }
        for(unsigned int i=0; i<n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException& e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }
        }

        commands_buffer_.writeFromNonRT(std::vector<PosVelAccEff>(n_joints_, PosVelAccEff()));

        sub_command_ = n.subscribe<trajectory_msgs::JointTrajectoryPoint>("command", 1, &ForwardJointTrajectoryPointController::commandCB, this);
        return true;
    }

//    void starting(const ros::Time& time);
//    virtual void update(const ros::Time &, const ros::Duration &) = 0;

    std::vector< std::string > joint_names_;
    std::vector< HandleType > joints_;
    realtime_tools::RealtimeBuffer<std::vector<PosVelAccEff> > commands_buffer_;
    unsigned int n_joints_;

protected:
    ros::Subscriber sub_command_;

    virtual void commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr&) = 0;

};

}

#endif
