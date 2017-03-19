#include "GzSimplePlugin.h"
#include <iostream>

namespace gazebo
{
GzSimplePlugin::GzSimplePlugin():ModelPlugin()
{
}

void GzSimplePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

    // Safety check
    if (_model->GetJointCount() == 0)
    {
        std::cerr << "Invalid joint count, plugin not loaded\n";
        return;
    }

    std::cout << "Plugin Loaded\n";


    //Pega parãmetros por conveniência
    this->model = _model;

    this->jointController = this->model->GetJointController();

    this->joints = this->model->GetJoints();

    // cria o nó
    this->node = transport::NodePtr(new transport::Node());

    this->node->Init(this->model->GetWorld()->GetName());

    const std::string writeRequestTopicName = "~/" + this->model->GetName() + "/writeRequest";

    const std::string readRequestTopicName = "~/" + this->model->GetName() + "/readRequest";

    const std::string readResponseTopicName = "~/" + this->model->GetName() + "/readResponse";

    // Subscribe to the topic, and register a callback
    this->writeRequestSub = this->node->Subscribe(writeRequestTopicName, &GzSimplePlugin::handleWriteRequest, this);

    this->readRequestSub = this->node->Subscribe(readRequestTopicName, &GzSimplePlugin::handleReadRequest, this);

    gazebo::transport::PublisherPtr pub = node->Advertise<gz_msgs::GzReadResponse>(readResponseTopicName);

    this->pubMap[pub->GetTopic()] = pub;

    for(auto joint: joints) {
        joint->SetProvideFeedback(true);
    }
}

void GzSimplePlugin::handleWriteRequest(GzWriteRequestPtr &msg)
{
    std::cout << "WriteRequest" << std::endl;

    std::cout << std::to_string(msg->jointids_size()) << std::endl;

    if(msg->jointids_size() > 0) {

        if(msg->jointids_size() == msg->pospid_size())
            setPosPids(msg);

        if(msg->jointids_size() == msg->velpid_size())
            setVelPids(msg);

        if(msg->jointids_size() == msg->pos_size() && msg->pos_size() == msg->vel_size()) {
            setPositions(msg);
            setVelocities(msg);
        }else if (msg->jointids_size() == msg->torque_size())
            setTorques(msg);
    }
}

void GzSimplePlugin::handleReadRequest(GzReadRequestPtr &msg) {

    std::cout << "readRequest" << std::endl;

    if(msg->jointids_size() > 0){

        gz_msgs::GzReadResponse response;

        for(int i = 0; i < msg->jointids_size(); i++){
            response.add_jointnames(msg->jointnames(i));

//            response->add_vel(joints[i]->GetVelocity(0));

//            std::cout << "GetVelocity of Joint " << response->jointids(i) << ": "<< response->vel(i) << std::endl;

//            gazebo::physics::JointWrench wrench = joints.at(i)->GetForceTorque(0);

//            response->add_torque(wrench.body1Torque.X()+wrench.body1Torque.Y()+wrench.body1Torque.Z());

//            std::cout << std::to_string(wrench.body1Torque.X()) + " " + std::to_string(wrench.body1Torque.Y()) + " " +std::to_string(wrench.body1Torque.Z())<< std::endl;
//            std::cout << std::to_string(wrench.body2Torque.X()) + " " + std::to_string(wrench.body2Torque.Y()) + " " +std::to_string(wrench.body2Torque.Z())<< std::endl;

//            std::cout << "GetTorque of Joint " << response->jointids(i) << ": "<< response->torque(i) << std::endl;
//            if(msg->has_pos()){
//                response->add_pos(joints[i]->GetAngle(0).Radian());
//                std::cout << "GetPosition of Joint " << response->jointids(i) << ": "<< response->pos(i) << std::endl;
//            }

        }

        if(msg->requestitem()==msg->POS_VEL_TORQUE && msg->jointids_size() > 0) {

            gz_msgs::GzReadResponse response;

            for(int i = 0; i < msg->jointids_size(); i++)
                response.add_jointids(msg->jointids(i));

            this->getJointStates(&response);

            transport::PublisherPtr pub;

            try {
                pub = pubMap.at(msg->returntopic());
            }catch(const std::out_of_range& ex) {
                std::cout << "topic não reconhecido: " +  msg->returntopic() << std::endl;
                pub = node->Advertise<gz_msgs::GzReadResponse>(msg->returntopic());
                pubMap[msg->returntopic()] = pub;
                std::cout << "topico criado e registrado" << std::endl;
            }

            pub->Publish(response, false);
            std::cout << "mensagem enviada de volta" << std::endl;

        }
    }

}


void GzSimplePlugin::setPositions(GzWriteRequestPtr &msg){
    for(int i = 0; i < msg->jointids_size(); i++){
        this->jointController->SetPositionTarget(this->joints.at(msg->jointids(i))->GetScopedName(),msg->pos(i));
        std::cout << "SetPosition of Joint " << msg->jointids(i) << ": "<< msg->pos(i) << std::endl;
    }
}

void GzSimplePlugin::setVelocities(GzWriteRequestPtr &msg){
    for(int i = 0; i < msg->jointids_size(); i++){
        this->jointController->SetVelocityTarget(this->joints.at(msg->jointids(i))->GetScopedName(),msg->vel(i));
        std::cout << "SetVelocity of Joint " << msg->jointids(i) << ": "<< msg->vel(i) << std::endl;

    }
}

void GzSimplePlugin::setTorques(GzWriteRequestPtr &msg){
    for(int i = 0; i < msg->jointids_size(); i++){
        this->joints.at(msg->jointids(i))->SetForce(0,msg->torque(i));
        std::cout << "SetTorque of Joint " << msg->jointids(i) << ": "<< msg->torque(i) << std::endl;

    }
}

std::string gzPidToString(gazebo::common::PID pid) {
    return "P = " + std::to_string(pid.GetPGain()) + "; I = " + std::to_string(pid.GetIGain()) + "; D = " + std::to_string(pid.GetDGain()) + ";";
}

void GzSimplePlugin::setPosPids(GzWriteRequestPtr& msg)
{
    for (int i = 0; i < msg->jointids_size(); i++) {
        gazebo::common::PID gzPid(msg->pospid(i).kp(), msg->pospid(i).ki(), msg->pospid(i).kd());
        jointController->SetPositionPID(this->joints.at(msg->jointids(i))->GetScopedName(),gzPid);
        std::cout << "SetPositionPID of Joint " << msg->jointids(i) << ": "<< gzPidToString(gzPid) << std::endl;
    }
}

void GzSimplePlugin::setVelPids(GzWriteRequestPtr& msg)
{
    for (int i = 0; i < msg->jointids_size(); i++) {
        gazebo::common::PID gzPid(msg->velpid(i).kp(), msg->velpid(i).ki(), msg->velpid(i).kd());
        jointController->SetVelocityPID(this->joints.at(msg->jointids(i))->GetScopedName(),gzPid);
        std::cout << "SetVelocityPID of Joint " << msg->jointids(i) << ": "<< gzPidToString(gzPid) << std::endl;
    }
}

void GzSimplePlugin::getJointStates(gz_msgs::GzReadResponse *response) {

    for (int i = 0; i < response->jointids_size(); i++) {

        response->add_pos(joints[i]->GetAngle(0).Radian());

        std::cout << "GetPosition of Joint " << response->jointids(i) << ": "<< response->pos(i) << std::endl;

        response->add_vel(joints[i]->GetVelocity(0));

        std::cout << "GetVelocity of Joint " << response->jointids(i) << ": "<< response->vel(i) << std::endl;

        gazebo::physics::JointWrench wrench = joints.at(i)->GetForceTorque(0);

        response->add_torque(wrench.body1Torque.X()+wrench.body1Torque.Y()+wrench.body1Torque.Z());

        std::cout << std::to_string(wrench.body1Torque.X()) + " " + std::to_string(wrench.body1Torque.Y()) + " " +std::to_string(wrench.body1Torque.Z())<< std::endl;
        std::cout << std::to_string(wrench.body2Torque.X()) + " " + std::to_string(wrench.body2Torque.Y()) + " " +std::to_string(wrench.body2Torque.Z())<< std::endl;

        std::cout << "GetTorque of Joint " << response->jointids(i) << ": "<< response->torque(i) << std::endl;
    }

}

}
