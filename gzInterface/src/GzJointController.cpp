
#include <GzJointController.h>
#include <gazebo/transport/transport.hh>
#include <GzWriteRequest.pb.h>
#include <gazebo/gazebo_client.hh>
#include <memory>


GzJointController::GzJointController(std::vector<std::__cxx11::string> jointNames, std::string pubTopic, std::string subTopic) : AbsPidJointController(jointNames) {

    gazebo::client::setup();//TODO: verificar se é melhor aqui ou fora

    // Create our node for communication
    gazebo::transport::NodePtr node = gazebo::transport::NodePtr(new gazebo::transport::Node());

    node->Init();

    // Publish to the  velodyne topic
    pub = node->Advertise<gz_msgs::GzWriteRequest>(pubTopic);

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Subscribe to the topic, and register a callback
    sub = node->Subscribe(subTopic, &GzJointController::onReadMsg, this);

}



GzJointController::~GzJointController()
{
    gazebo::client::shutdown();//TODO: verificar se é melhor aqui ou fora
}



bool GzJointController::goPosVel(double pos, double vel, int jointID) {

    gz_msgs::GzWriteRequest msg;


    msg.add_jointids(jointID);

    msg.add_pos(pos);

    msg.add_vel(vel);

    pub->Publish(msg, false);

    return true;
}

bool GzJointController::goPosVel(double pos, double vel, std::__cxx11::string jointName) {

    return this->goPosVel(pos, vel, this->jointNamesMap[jointName]);

}

bool GzJointController::goPosVel(std::vector<JointPosVelCommand> cmd) {

    gz_msgs::GzWriteRequest msg;

    for(unsigned int i = 0; i < cmd.size(); i++) {
        msg.add_jointids(i);
        msg.add_pos(cmd[i].getPos());
        msg.add_vel(cmd[i].getVel());
    }

    pub->Publish(msg, false);

    return true;

}

bool GzJointController::goTorque(double torque, int jointID) {

    gz_msgs::GzWriteRequest msg;

    msg.add_jointids(jointID);

    msg.add_torque(torque);

    pub->Publish(msg, false);

    return true;

}

bool GzJointController::goTorque(double torque, std::__cxx11::string jointName) {

    return goTorque(torque,this->jointNamesMap[jointName]);

}

bool GzJointController::goTorque(std::vector<JointTorqueCommand> cmd) {

    gz_msgs::GzWriteRequest msg;

    for(unsigned int i = 0; i < cmd.size(); i++) {
        msg.add_jointids(i);
        msg.add_torque(cmd[i].getTorque());
    }

    pub->Publish(msg, false);

    return true;

}

bool GzJointController::readJointStates() {
    return true;

}

void GzJointController::onReadMsg(GzReadResponsePtr& msg)
{

}

void addPidWriteMsg(gz_msgs::GzWriteRequest* msg, gz_msgs::PidRequest* pidMsg, PidValues pid) {
    pidMsg->set_kp(pid.kp);
    pidMsg->set_ki(pid.ki);
    pidMsg->set_kd(pid.kd);

}


bool GzJointController::setPosPid(PidValues pid, int jointID)
{
    gz_msgs::GzWriteRequest msg;

    gz_msgs::PidRequest* pidMsg = msg.add_pospid();

    addPidWriteMsg(&msg, pidMsg, pid);

    msg.add_jointids(jointID);

    pub->Publish(msg, false);

    AbsPidJointController::setPosPid(pid, jointID);

    return true;
}

bool GzJointController::setPosPid(PidValues pid, std::__cxx11::string jointName)
{

    return setPosPid(pid,this->jointNamesMap[jointName]);

}

bool GzJointController::setPosPid(std::vector<PidValues> pids)
{
    gz_msgs::GzWriteRequest msg;

    for (unsigned int i = 0; i < pids.size(); i++) {
        gz_msgs::PidRequest* pidMsg = msg.add_pospid();
        addPidWriteMsg(&msg, pidMsg,pids[i]);
        msg.add_jointids(i);
    }

    pub->Publish(msg, false);

    AbsPidJointController::setPosPid(pids);

    return true;
}

bool GzJointController::setVelPid(PidValues pid, int jointID)
{

    gz_msgs::GzWriteRequest msg;

    gz_msgs::PidRequest* pidMsg = msg.add_velpid();

    addPidWriteMsg(&msg, pidMsg, pid);

    msg.add_jointids(jointID);

    pub->Publish(msg, false);

    AbsPidJointController::setVelPid(pid, jointID);

    return true;
}

bool GzJointController::setVelPid(PidValues pid, std::__cxx11::string jointName)
{

    return    setVelPid(pid,this->jointNamesMap[jointName]);

}

bool GzJointController::setVelPid(std::vector<PidValues> pids)
{

    gz_msgs::GzWriteRequest msg;

    for (unsigned int i = 0; i < pids.size(); i++) {
        gz_msgs::PidRequest* pidMsg = msg.add_velpid();
        addPidWriteMsg(&msg, pidMsg,pids[i]);
        msg.add_jointids(i);
    }

    pub->Publish(msg, false);

    AbsPidJointController::setVelPid(pids);

    return true;
}

bool GzJointController::setPosVelPid(PidValues posPid, PidValues velPid, int jointID)
{

    gz_msgs::GzWriteRequest msg;

    gz_msgs::PidRequest* posPidMsg = msg.add_pospid();

    addPidWriteMsg(&msg, posPidMsg, posPid);

    //std::cout << std::to_string(posPid.kp) + " " + std::to_string(msg.pospid(0).kp()) + " " + std::to_string(posPidMsg->kp()) << std::endl;

    gz_msgs::PidRequest* velPidMsg = msg.add_velpid();

    addPidWriteMsg(&msg, velPidMsg, velPid);

    msg.add_jointids(jointID);

    pub->Publish(msg, false);

    AbsPidJointController::setPosVelPid(posPid, velPid, jointID);

    return true;

}

bool GzJointController::setPosVelPid(PidValues posPid, PidValues velPid, std::__cxx11::string jointName)
{

    return setPosVelPid(posPid, velPid, this->jointNamesMap[jointName]);

}

bool GzJointController::setPosVelPid(std::vector<PidValues> posPids, std::vector<PidValues> velPids)
{

    if(posPids.size() == velPids.size()) {
        gz_msgs::GzWriteRequest msg;

        for (unsigned int i = 0; i < posPids.size(); i++) {
            gz_msgs::PidRequest* velPidMsg = msg.add_velpid();
            addPidWriteMsg(&msg, velPidMsg,velPids[i]);
            gz_msgs::PidRequest* posPidMsg = msg.add_pospid();
            addPidWriteMsg(&msg, posPidMsg,posPids[i]);
            msg.add_jointids(i);
        }

        pub->Publish(msg, false);

        AbsPidJointController::setPosVelPid(posPids, velPids);

        return true;
    }

    return false;
}
