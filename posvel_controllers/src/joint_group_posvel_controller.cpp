#include <pluginlib/class_list_macros.h>
#include <joint_group_posvel_controller.h>

template <class T>
void forward_trajectory_point_controller::ForwardJointTrajectoryPointController<T>::update(const ros::Time &, const ros::Duration &) {
    std::vector<PosVelAccEff> & commands = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {  joints_[i].setCommand(commands[i].position,commands[i].velocity);  }
}

template <class T>
void forward_trajectory_point_controller::ForwardJointTrajectoryPointController<T>::commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr &msg){

    if(msg->positions.size() != n_joints_ || msg->velocities.size() != n_joints_)
    {
        ROS_ERROR_STREAM("Dimension of command positions (" << msg->positions.size() << ") or velocities(" << msg->velocities.size() <<
                         ") do not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }

    std::vector<PosVelAccEff> values;

    for(unsigned int i = 0; i < n_joints_; i++)
        values.push_back(PosVelAccEff(msg->positions[i],msg->velocities[i],0,0));

    commands_buffer_.writeFromNonRT(values);
}

PLUGINLIB_EXPORT_CLASS(posvel_controllers::JointGroupPosVelController, controller_interface::ControllerBase);
