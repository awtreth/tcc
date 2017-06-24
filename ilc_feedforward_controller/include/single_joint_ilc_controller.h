#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

using namespace hardware_interface;

class SingleJointILCController:
        public controller_interface::Controller<hardware_interface::PositionJointInterface>{

    // ControllerBase interface
public:

    void update(const ros::Time& now, const ros::Duration&);

    bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle & nh);

    void starting(const ros::Time& time);

    void setReferenceTrajectory(std::vector<double> positions);

    void setLearningFactor(double learning_factor);

    //TODO: not implemented yet
    void setDelayTime(double delay_time);

    int runIteration();

    std::vector<double> getTimestamps() const;

    std::vector<double> getReference() const;

    std::vector<double> getInput() const;

    std::vector<double> getOutput() const;

    std::vector<double> getErrors() const;


    void reset();

private:
    hardware_interface::JointHandle joint;

    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> >ref_pub;
    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> >input_pub;
    double publish_rate = 50;

    ros::Time last_publish_time_;
    ros::Time start_time;

    double factor = .1;
    double delay = 0;

    bool running = false;
    std::mutex mtx;
    std::condition_variable cv;

    std::vector<double> reference_positions;
    std::vector<double> timestamps;
    std::vector<double> input_positions;
    std::vector<double> output_positions;
    std::vector<double> errors;
    int next_iteration = 0;

    size_t curr_pos = 0;

    void computeInputPositions();

    void computeErrors();

};

