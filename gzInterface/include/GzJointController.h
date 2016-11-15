#ifndef GZJOINTCONTROLLER_H
#define GZJOINTCONTROLLER_H

#include <JointController.h>
#include <string>
#include <gazebo/transport/transport.hh>
#include <GzReadAnswer.pb.h>
#include <memory>

class GzJointController : public AbsJointController, public IJointPosVelController, public IJointTorqueController {

    private:

    gazebo::transport::PublisherPtr pub;
    gazebo::transport::SubscriberPtr sub;

    typedef const boost::shared_ptr<const gz_msgs::ReadAnswer> ReadAnswerPtr;

    void onReadMsg(ReadAnswerPtr &msg);

    public:

    GzJointController(std::vector<std::string> jointNames, std::string pubTopic, std::string subTopic);

    ~GzJointController();

    // IJointTorqueController interface
    public:
    bool goTorque(double torque, int jointID) override;
    bool goTorque(double torque, std::__cxx11::string jointName) override;
    bool goTorque(std::vector<JointTorqueCommand>) override;

    // IJointPosVelController interface
    public:
    bool goPosVel(double pos, double vel, int jointID) override;
    bool goPosVel(double pos, double vel, std::__cxx11::string jointName) override;
    bool goPosVel(std::vector<JointPosVelCommand>) override;

    // AbsJointController interface
    public:
    bool readJointStates() override;
};

#endif
