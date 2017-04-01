
#include <GzJointController.h>
#include <gazebo/transport/transport.hh>
#include <GzWriteRequest.pb.h>
#include <GzReadRequest.pb.h>
#include <gazebo/gazebo_client.hh>
#include <memory>
#include <cstring>


GzJointController::GzJointController(std::vector<std::__cxx11::string> jointNames, std::string pubWriteTopic, std::string pubReadTopic, std::string subTopic) : AbsJointController(jointNames) {

    gazebo::client::setup();//TODO: verificar se é melhor aqui ou fora

    // Create our node for communication
    gazebo::transport::NodePtr node = gazebo::transport::NodePtr(new gazebo::transport::Node());

    node->Init();

    // Publish to the  velodyne topic
    writePub = node->Advertise<gz_msgs::GzWriteRequest>(pubWriteTopic);
    readPub = node->Advertise<gz_msgs::GzReadRequest>(pubReadTopic);


    // Wait for a subscriber to connect to this publisher
    writePub->WaitForConnection();
    readPub->WaitForConnection();

    // Subscribe to the topic, and register a callback
    sub = node->Subscribe(subTopic, &GzJointController::onReadMsg, this);

    for(auto name : jointNames)
        jointMap[name] = Joint(name);

}



GzJointController::~GzJointController()
{
    gazebo::client::shutdown();//TODO: verificar se é melhor aqui ou fora
}

bool GzJointController::sendTorqueCommand()
{
    return sendCommand();//FIXME: enviar somente Torque
}

bool GzJointController::addTorqueCommand(TorqueWriteJointCommand cmd)
{
    writeMsgMtx.lock();
    writeCmd.add_jointnames(cmd.getJointName());
    writeCmd.add_torque(cmd.getTorque());
    writeMsgMtx.unlock();
    return true;
}

bool GzJointController::sendPosVelCommand()
{
    return sendCommand();//FIXME: enviar somente PosVel
}

bool GzJointController::sendPosVelCommand(std::vector<PosVelWriteJointCommand> cmd)
{
    for(auto command : cmd)
        addPosVelCommand(command);

    return sendPosVelCommand();
}

bool GzJointController::addPosVelCommand(PosVelWriteJointCommand cmd)
{
    writeMsgMtx.lock();
    writeCmd.add_jointnames(cmd.getJointName());
    writeCmd.add_pos(cmd.getPos());
    writeCmd.add_vel(cmd.getVel());
    writeMsgMtx.unlock();
    return true;
}

bool GzJointController::sendRequest()
{
    return sendReadMsg();
}

bool GzJointController::sendRequest(std::vector<ReadJointCommand> cmds)
{
    for(auto command : cmds)
        addRequest(command);

    return sendReadMsg();
}

bool GzJointController::addRequest(ReadJointCommand cmd)
{
    readMsgMtx.lock();
    readCmd.add_jointnames(cmd.getJointName());
    //TODO: separar Pos Vel
    readCmd.set_pos(cmd.hasPosVel());
    readCmd.set_vel(cmd.hasPosVel());
    readCmd.set_torque(cmd.hasTorque());
    //TODO: separar PosPid e VelPid
    readCmd.set_posvelpid(cmd.hasPosVelPid());
    readMsgMtx.unlock();
    return true;
}

JointMap GzJointController::getLastJointState()
{
    std::unique_lock<std::mutex> lk(waitResponseMtx);
    if(waitingResponse)
        waitResponseCv.wait(lk);

    auto jMapCopy = jointMap;
    waitResponseMtx.unlock();

    return jMapCopy;
}

bool GzJointController::sendCommand()
{
    return sendWriteMsg();
}

bool GzJointController::sendCommand(std::vector<WriteJointCommand> cmds)
{
    for(auto command : cmds)
        addCommand(command);

    return sendCommand();
}

bool GzJointController::sendTorqueCommand(std::vector<TorqueWriteJointCommand> cmd)
{

    for(auto command : cmd)
        addTorqueCommand(command);

    return sendTorqueCommand();
}

bool GzJointController::addCommand(WriteJointCommand cmd)
{

    if(strcmp(cmd.getCmdID(),PosVelWriteJointCommand::CMD_ID)==0){
        return addPosVelCommand(static_cast<PosVelWriteJointCommand&>(cmd));
    }else if(strcmp(cmd.getCmdID(),TorqueWriteJointCommand::CMD_ID)==0){
        return addTorqueCommand(static_cast<TorqueWriteJointCommand&>(cmd));
    }

    return false;
}


//bool GzJointController::readJointStates() {

//    gz_msgs::GzReadRequest msg;

//    msg.set_returntopic(sub->GetTopic());

//    msg.set_requestitem(msg.POS_VEL_TORQUE);

//    for(unsigned int i = 0; i < this->jointVec.size(); i++)
//        msg.add_jointids(i);

//    readPub->Publish(msg, false);

//    return true;

//}

void GzJointController::onReadMsg(GzReadResponsePtr& msg)
{
    for(int i = 0; i < msg->jointnames_size(); i++) {

        waitResponseMtx.lock();
        if(msg->pos_size()>0)
            jointMap.at(msg->jointnames(i)).setPos(msg->pos(i));

        if(msg->vel_size()>0)
            jointMap.at(msg->jointnames(i)).setVel(msg->vel(i));

        if(msg->torque_size()>0)
            jointMap.at(msg->jointnames(i)).setTorque(msg->torque(i));

        if(msg->pospid_size()>0)
            jointMap.at(msg->jointnames(i)).setPosPid(msg->pospid(i).kp(), msg->pospid(i).kd(), msg->pospid(i).ki());

        if(msg->velpid_size()>0)
            jointMap.at(msg->jointnames(i)).setVelPid(msg->velpid(i).kp(), msg->velpid(i).kd(), msg->velpid(i).ki());

        waitingResponse = false;
        waitResponseMtx.unlock();
        waitResponseCv.notify_all();

//        std::cout << "Joint " + std::to_string(i) + ": " + jointMap.at(msg->jointnames(i)).getJointState().toString() << std::endl;
    }

}

bool GzJointController::sendWriteMsg()
{
    writeMsgMtx.lock();
    if(writeCmd.jointnames_size()>0){
        gz_msgs::GzWriteRequest msg = writeCmd;//faz uma copia
        writeCmd = gz_msgs::GzWriteRequest();
        writeMsgMtx.unlock();
        writePub->Publish(msg, false);
        return true;
    }else{
        writeMsgMtx.unlock();
        return false;
    }
}

bool GzJointController::sendReadMsg()
{
    readMsgMtx.lock();
    waitResponseMtx.lock();

    if(readCmd.jointnames_size()>0 && !waitingResponse){
        waitingResponse = true;
        gz_msgs::GzReadRequest msg = readCmd;//faz uma copia
        readCmd = gz_msgs::GzReadRequest();

        waitResponseMtx.unlock();
        readMsgMtx.unlock();

        readPub->Publish(msg, false);
        return true;
    }else{
        waitResponseMtx.unlock();
        readMsgMtx.unlock();
        return false;
    }

}

//void addPidWriteMsg(gz_msgs::GzWriteRequest* msg, gz_msgs::PidRequest* pidMsg, PidValues pid) {
//    pidMsg->set_kp(pid.kp);
//    pidMsg->set_ki(pid.ki);
//    pidMsg->set_kd(pid.kd);
//}

//bool GzJointController::setPosPid(std::vector<PidValues> pids)
//{
//    gz_msgs::GzWriteRequest msg;

//    for (unsigned int i = 0; i < pids.size(); i++) {
//        gz_msgs::PidRequest* pidMsg = msg.add_pospid();
//        addPidWriteMsg(&msg, pidMsg,pids[i]);
//        msg.add_jointids(i);
//    }

//    writePub->Publish(msg, false);

//    AbsPidJointController::setPosPid(pids);

//    return true;
//}


//bool GzJointController::setVelPid(std::vector<PidValues> pids)
//{

//    gz_msgs::GzWriteRequest msg;

//    for (unsigned int i = 0; i < pids.size(); i++) {
//        gz_msgs::PidRequest* pidMsg = msg.add_velpid();
//        addPidWriteMsg(&msg, pidMsg,pids[i]);
//        msg.add_jointids(i);
//    }

//    writePub->Publish(msg, false);

//    AbsPidJointController::setVelPid(pids);

//    return true;
//}

//bool GzJointController::setPosVelPid(std::vector<PidValues> posPids, std::vector<PidValues> velPids)
//{

//    if(posPids.size() == velPids.size()) {
//        gz_msgs::GzWriteRequest msg;

//        for (unsigned int i = 0; i < posPids.size(); i++) {
//            gz_msgs::PidRequest* velPidMsg = msg.add_velpid();
//            addPidWriteMsg(&msg, velPidMsg,velPids[i]);
//            gz_msgs::PidRequest* posPidMsg = msg.add_pospid();
//            addPidWriteMsg(&msg, posPidMsg,posPids[i]);
//            msg.add_jointids(i);
//        }

//        writePub->Publish(msg, false);

//        AbsPidJointController::setPosVelPid(posPids, velPids);

//        return true;
//    }

//    return false;
//}
