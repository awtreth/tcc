#include "GzDxlPlugin.h"
#include <iostream>

namespace gazebo
{
    GzDxlPlugin::GzDxlPlugin():ModelPlugin()
    {
    }

    void GzDxlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

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

        for(int i = 0; i < this->model->GetJointCount(); i++){
            std::string jointName = this->joints.at(i)->GetScopedName();
            this->jointController->SetVelocityPID(jointName,defaultPID);
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
                                          &GzDxlPlugin::HandleCommand, this);

    }

    void GzDxlPlugin::HandleCommand(GzDxlRequestPtr &_msg)
    {
        std::cout << std::to_string(_msg->nmotors()) << std::endl;
        //this->SetVelocity(_msg->x());
    }

}
