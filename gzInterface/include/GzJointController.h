#ifndef GZJOINTCONTROLLER_H
#define GZJOINTCONTROLLER_H

#include <JointController.h>
#include <string>
#include <gazebo/transport/transport.hh>
#include <GzReadResponse.pb.h>
#include <memory>

class GzJointController : public AbsPidJointController, public IPosVelJointController, public ITorqueJointController {

    private:

    gazebo::transport::PublisherPtr writePub;
    gazebo::transport::PublisherPtr readPub;
    gazebo::transport::SubscriberPtr sub;

    typedef const boost::shared_ptr<const gz_msgs::GzReadResponse> GzReadResponsePtr;

    void onReadMsg(GzReadResponsePtr &msg);

    public:

    GzJointController(std::vector<std::string> jointNames, std::string pubWriteTopic, std::string pubReadTopic, std::string subTopic);

    ~GzJointController();

    // IJointTorqueController interface
    public:
    bool goTorque(double torque, int jointID) override;
    bool goTorque(double torque, std::__cxx11::string jointName) override;
    bool goTorque(std::vector<TorqueWriteJointCommand>) override;

    // IJointPosVelController interface
    public:
    bool goPosVel(double pos, double vel, int jointID) override;//testado
    bool goPosVel(double pos, double vel, std::__cxx11::string jointName) override;//testado
    bool goPosVel(std::vector<PosVelWriteJointCommand>) override;//testado

    // AbsJointController interface
    public:
    bool readJointStates() override;


    // AbsPidJointController interface
    public:
    virtual bool setPosPid(PidValues pid, int jointID);
    virtual bool setPosPid(PidValues pid, std::__cxx11::string jointName);
    virtual bool setPosPid(std::vector<PidValues> pids);

    virtual bool setVelPid(PidValues pid, int jointID);
    virtual bool setVelPid(PidValues pid, std::__cxx11::string jointName);
    virtual bool setVelPid(std::vector<PidValues> pids);

    virtual bool setPosVelPid(PidValues posPid, PidValues velPid, int jointID);//testado
    virtual bool setPosVelPid(PidValues posPid, PidValues velPid, std::__cxx11::string jointName);//testado
    virtual bool setPosVelPid(std::vector<PidValues> posPids, std::vector<PidValues> velPids);//testado

    // AbsJointController interface
    virtual bool sendWriteCommand(std::vector<WriteJointCommandPtr> cmd);
    std::vector<Joint> sendReadCommand(std::vector<ReadJointCommandPtr> cmd);
};

#endif
