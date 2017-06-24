#include <single_joint_ilc_controller.h>
#include <ros/ros.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

void SingleJointILCController::update(const ros::Time &now, const ros::Duration &){
    if(running){
        if(curr_pos < reference_positions.size()){
            ros::Duration duration_from_start = now - start_time;
            timestamps[curr_pos] = duration_from_start.sec + duration_from_start.nsec/1e9;

            output_positions[curr_pos] = joint.getPosition();

            if (publish_rate > 0.0 && (last_publish_time_ + ros::Duration(1.0/publish_rate)) < now){
                if (ref_pub->trylock()){
                    last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate);
                    ref_pub->msg_.data = reference_positions[curr_pos];
                    ref_pub->unlockAndPublish();
                }
                if(input_pub->trylock()){
                    last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate);
                    input_pub->msg_.data = input_positions[curr_pos];
                    input_pub->unlockAndPublish();
                }
            }

            joint.setCommand(input_positions[curr_pos++]);

        }else{
            std::unique_lock<std::mutex> lck(mtx);
            running = false;
            lck.unlock();
            cv.notify_all();
        }
    }
}

bool SingleJointILCController::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh){
    joint = hw->getHandle("joint");
    ref_pub.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh, "ilc_ref",1));
    input_pub.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh, "ilc_input",1));

    ref_pub->msg_.data = 0;
    input_pub->msg_.data = 0;

    return true;
}

void SingleJointILCController::starting(const ros::Time &time)
{
    // initialize time
    last_publish_time_ = time;
}

void SingleJointILCController::setReferenceTrajectory(std::vector<double> positions){
    reference_positions = positions;
    reset();
}

void SingleJointILCController::setLearningFactor(double learning_factor){
    factor = learning_factor;
}

void SingleJointILCController::setDelayTime(double delay_time){
    delay = delay_time;
}

int SingleJointILCController::runIteration(){
    start_time = ros::Time::now();
    running = true;

    std::unique_lock<std::mutex> lck(mtx);
    cv.wait(lck);
    curr_pos = 0;
    lck.unlock();

    computeErrors();
    computeInputPositions();

    return ++next_iteration;
}

std::vector<double> SingleJointILCController::getTimestamps() const{
    return timestamps;
}

std::vector<double> SingleJointILCController::getReference() const{
    return reference_positions;
}

std::vector<double> SingleJointILCController::getInput() const{
    return input_positions;
}

std::vector<double> SingleJointILCController::getOutput() const{
    return output_positions;
}

std::vector<double> SingleJointILCController::getErrors() const{
    return errors;
}

void SingleJointILCController::reset(){
    input_positions = reference_positions;
    next_iteration = 0;

    errors = std::vector<double>(reference_positions.size(),0);
    timestamps = errors;
    output_positions = errors;
}

void SingleJointILCController::computeInputPositions(){
    for(size_t i = 0; i < reference_positions.size(); i++)
        input_positions[i] += factor*errors[i];
}

void SingleJointILCController::computeErrors(){
    for(size_t i = 0; i < reference_positions.size(); i++)
        errors[i] = reference_positions[i]-output_positions[i];
}

PLUGINLIB_EXPORT_CLASS(SingleJointILCController, controller_interface::ControllerBase);
