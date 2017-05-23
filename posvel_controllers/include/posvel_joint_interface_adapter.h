#ifndef JOINT_TRAJECTORY_CONTROLLER_POSVEL_JOINT_INTERFACE_ADAPTER_H
#define JOINT_TRAJECTORY_CONTROLLER_POSVEL_JOINT_INTERFACE_ADAPTER_H

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <hardware_interface/posvel_command_interface.h>

template <class State>
class HardwareInterfaceAdapter<hardware_interface::PosVelJointInterface, State>
{
public:
    HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

    bool init(std::vector<hardware_interface::PosVelJointHandle>& joint_handles, ros::NodeHandle& /*controller_nh*/)
    {
        // Store pointer to joint handles
        joint_handles_ptr_ = &joint_handles;

        return true;
    }

    void starting(const ros::Time& /*time*/)
    {
        if (!joint_handles_ptr_) {return;}

        // Semantic zero for commands
        for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i)
        {
            (*joint_handles_ptr_)[i].setCommand((*joint_handles_ptr_)[i].getPosition(),0);
        }
    }

    void stopping(const ros::Time& /*time*/) {}

    void updateCommand(const ros::Time&     /*time*/,
                       const ros::Duration& /*period*/,
                       const State&         desired_state,
                       const State&         /*state_error*/)
    {
        // Forward desired position and velocity to command
        const unsigned int n_joints = joint_handles_ptr_->size();
        for (unsigned int i = 0; i < n_joints; ++i) {
            (*joint_handles_ptr_)[i].setCommand(desired_state.position[i],desired_state.velocity[i]);
        }
    }

private:
    std::vector<hardware_interface::PosVelJointHandle>* joint_handles_ptr_;
};


#endif
