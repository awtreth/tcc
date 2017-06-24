#include <single_joint_ilc_controller.h>
#include <iostream>
#include <vector>
#include <ControlTimer.h>
#include <RosControllerManagerAdapter.h>
#include <DxlRobotHW.h>
#include <controller_manager/controller_manager.h>
#include <cmath>
#include <limits.h>

#define FREQUENCY 50.
#define TOTAL_TIME 10.
#define RAD_DEGREE_RATIO (M_PI/180.)

double sample_trajectory(double t){
    return .5*(sin(2*t) - sin(4*t));
}

std::vector<double> build_trajectory(double period, double total_time){
    size_t size = size_t(lround(total_time/period));

    std::vector<double> positions(size,0);

    for(size_t i = 0; i < size; i++)
        positions[i] = sample_trajectory(i*period);

    return positions;
}

double getMax(std::vector<double> vec){
    double max = vec[0];
    for(auto el : vec){
        if(el>=max)
            max = el;
    }
    return max;
}

double getRMS(std::vector<double> vec){
    double sum = 0;

    for(auto el : vec)
        sum+= el*el;

    return sqrt(sum)/vec.size();
}

int main(int argc, char** argv){


    ros::init(argc, argv, "single_joint_ilc_controller_test");

    ros::NodeHandle nh;

    nh.setParam("/single_joint_ilc_controller/type", "ilc_feedforward_controllers/SingleJointILCController");
    nh.setParam("/joint_state_controller/type", "joint_state_controller/JointStateController");
    nh.setParam("/joint_state_controller/publish_rate", 50.);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::vector<JointID> jointIDs;

    jointIDs.push_back(JointID("joint", 1, 180*RAD_DEGREE_RATIO,1));

    auto hw = std::make_shared<DxlRobotHW>(jointIDs);

    auto cma = std::make_shared<RosControllerManagerAdapter>(hw.get(),nh);

    ControlTimer ctimer(hw);

    ctimer.setFrequency(FREQUENCY);
    ctimer.loadController("MyController",cma);

    ctimer.resumeLoop();

    std::cout << "trying to load controller" << std::endl;

    std::vector<std::string> controller_names;
    controller_names.push_back("single_joint_ilc_controller");
    controller_names.push_back("joint_state_controller");

    for(auto controller_name : controller_names)
        cma->getControllerManager().loadController(controller_name);

    cma->getControllerManager().switchController(controller_names,std::vector<std::string>(),controller_manager_msgs::SwitchController::Request::BEST_EFFORT);


    SingleJointILCController* controller = static_cast<SingleJointILCController*>(cma->getControllerManager().getControllerByName("single_joint_ilc_controller"));

    controller->setReferenceTrajectory(build_trajectory(1/FREQUENCY,TOTAL_TIME));


    while(ros::ok()){
        std::cout << "Press Enter to Run Iteration" << std::endl;
        getchar();

        if(!ros::ok())
            break;

        controller->runIteration();

        for(size_t i = 0; i < controller->getReference().size(); i++){
            std::cout << controller->getTimestamps()[i] << " " << controller->getReference()[i] << " ";
            std::cout << controller->getInput()[i] << " " << controller->getOutput()[i] << " " << controller->getErrors()[i] << std::endl;
        }

        std::cout << "errMAX: " << getMax(controller->getErrors()) << " errRMS: " << getRMS(controller->getErrors()) << std::endl;
    }

    return 0;

}
