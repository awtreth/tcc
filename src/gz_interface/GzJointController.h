#ifndef GZJOINTCONTROLLER_H
#define GZJOINTCONTROLLER_H

#include <AbsJointController.h>
#include <IPosVelJointController.h>
#include <ITorqueJointController.h>
#include <IWriteJointController.h>
#include <IReadJointController.h>
#include <string>
#include <gazebo/transport/transport.hh>
#include <GzReadResponse.pb.h>
#include <GzWriteRequest.pb.h>
#include <GzReadRequest.pb.h>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <Joint.h>

class GzJointController : public AbsJointController, public IWriteJointController,
        public IReadJointController, public IPosVelJointController, public ITorqueJointController {

private:

    gazebo::transport::PublisherPtr writePub;
    gazebo::transport::PublisherPtr readPub;
    gazebo::transport::SubscriberPtr sub;

    typedef const boost::shared_ptr<const gz_msgs::GzReadResponse> GzReadResponsePtr;

    void onReadMsg(GzReadResponsePtr &msg);

    gz_msgs::GzWriteRequest writeCmd;
    gz_msgs::GzReadRequest readCmd;

    bool sendWriteMsg();

    bool sendReadMsg();

    JointMap jointMap;

    bool waitingResponse = false;

    std::mutex readMsgMtx;
    std::mutex writeMsgMtx;
    std::mutex waitResponseMtx;

    std::condition_variable waitResponseCv;

public:

    GzJointController(std::vector<std::string> jointNames, std::string pubWriteTopic, std::string pubReadTopic, std::string subTopic);

    ~GzJointController();


    // ITorqueJointController interface
public:
    bool sendTorqueCommand();
    bool sendTorqueCommand(std::vector<TorqueWriteJointCommand> cmd);
    bool addTorqueCommand(TorqueWriteJointCommand cmd);

    // IPosVelJointController interface
public:
    bool sendPosVelCommand();
    bool sendPosVelCommand(std::vector<PosVelWriteJointCommand> cmd);
    bool addPosVelCommand(PosVelWriteJointCommand cmd);

    // IReadJointController interface
private:
    bool sendRequest();
    bool sendRequest(std::vector<ReadJointCommand> cmds);
    bool addRequest(ReadJointCommand cmd);
    JointMap getLastJointState();

    // IWriteJointController interface
public:
    bool sendCommand();
    bool sendCommand(std::vector<WriteJointCommand> cmds);
    bool addCommand(WriteJointCommand cmd);
};

#endif
