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

        this->readRequestSub = this->node->Subscribe(writeRequestTopicName, &GzSimplePlugin::handleReadRequest, this);

        //TODO: publish response
    }

    void GzSimplePlugin::handleWriteRequest(GzWriteRequestPtr &msg)
    {
        std::cout << "WriteRequest" << std::endl;
        std::cout << std::to_string(msg->jointids_size()) + " " + std::to_string(msg->pospid_size()) + " " + std::to_string(msg->velpid_size()) << std::endl;


        if(msg->jointids_size() > 0) {

            if(msg->jointids_size() == msg->pospid_size())
                setPosPids(msg);

            if(msg->jointids_size() == msg->velpid_size())
                setVelPids(msg);

            if(msg->jointids_size() == msg->pos_size() && msg->pos_size() == msg->vel_size())
                setPositions(msg);
            else if (msg->jointids_size() == msg->torque_size())
                setTorques(msg);
        }
    }

    void GzSimplePlugin::handleReadRequest(GzReadRequestPtr &msg) {

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

}
