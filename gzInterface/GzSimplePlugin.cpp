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

        // Store the model pointer for convenience.
        this->model = _model;

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->jointController = this->model->GetJointController();

        this->joints = this->model->GetJoints();

        const common::PID defaultPID = common::PID(0.1);

        for(unsigned int i = 0; i < this->model->GetJointCount(); i++){
            std::string jointName = this->joints.at(i)->GetScopedName();
            this->jointController->SetVelocityPID(jointName,common::PID(0));
            this->jointController->SetPositionPID(jointName,defaultPID);
            this->jointController->SetPositionTarget(jointName,0);
            this->jointController->SetVelocityTarget(jointName,0);
        }

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->GetName());

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/dxl_cmd";

        // Subscribe to the topic, and register a callback
        this->cmdSub = this->node->Subscribe(topicName,
                                             &GzSimplePlugin::HandleCommand, this);

    }

    void GzSimplePlugin::HandleCommand(GzSimpleRequestPtr &_msg)
    {
        if(_msg->requesttype()==gz_interface_msgs::msg::GzSimpleRequest_RequestType_WRITE){
            std::cout << "WRITE\n";
            switch(_msg->requestitem()){
                case gz_interface_msgs::msg::GzSimpleRequest_RequestItem_POS:
                    std::cout << "POS\n";
                    SetPositions(_msg);
                    break;
                case gz_interface_msgs::msg::GzSimpleRequest_RequestItem_VEL:
                    std::cout << "VEL\n";
                    SetVelocities(_msg);
                    break;
                case gz_interface_msgs::msg::GzSimpleRequest_RequestItem_POS_VEL:
                    std::cout << "POS_VEL\n";
                    SetPositions(_msg);
                    SetVelocities(_msg);
                    break;
                case gz_interface_msgs::msg::GzSimpleRequest_RequestItem_TORQUE:
                    std::cout << "TORQUE\n";
                    SetTorques(_msg);
                    break;
                default: std::cout << "switch(_msg->requestitem()): default\n";;
            }

        }else if(_msg->requesttype()==gz_interface_msgs::msg::GzSimpleRequest_RequestType_READ){
            std::cout << "READ\n";
        }

        std::cout << std::to_string(_msg->nmotors()) << std::endl;
        //this->SetVelocity(_msg->x());
    }

    void GzSimplePlugin::SetPositions(GzSimpleRequestPtr &_msg){
        for(int i = 0; i < _msg->motorid_size(); i++){
            this->jointController->SetPositionTarget(this->joints.at(_msg->motorid(i))->GetScopedName(),_msg->pos(i));
            std::cout << "SetPosition of Motor " << _msg->motorid(i) << ": "<< _msg->pos(i) << std::endl;
        }
    }

    void GzSimplePlugin::SetVelocities(GzSimpleRequestPtr &_msg){
        for(int i = 0; i < _msg->motorid_size(); i++){
            this->jointController->SetVelocityTarget(this->joints.at(_msg->motorid(i))->GetScopedName(),_msg->vel(i));
            std::cout << "SetVelocity of Motor " << _msg->motorid(i) << ": "<< _msg->vel(i) << std::endl;

        }
    }

    void GzSimplePlugin::SetTorques(GzSimpleRequestPtr &_msg){
        for(int i = 0; i < _msg->motorid_size(); i++){
            this->joints.at(_msg->motorid(i))->SetForce(0,_msg->torque(i));
            std::cout << "SetTorque of Motor " << _msg->motorid(i) << ": "<< _msg->torque(i) << std::endl;

        }
    }

}
