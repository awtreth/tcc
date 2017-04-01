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


    //Pega parâmetros por conveniência
    this->model = _model;

    this->jointController = this->model->GetJointController();

    this->joints = this->model->GetJoints();

    // cria o nó
    this->node = transport::NodePtr(new transport::Node());

#if GAZEBO_MAJOR_VERSION >= 8
    this->node->Init(this->model->GetWorld()->Name());
#else
    this->node->Init(this->model->GetWorld()->GetName());
#endif

    const std::string writeRequestTopicName = "~/" + this->model->GetName() + "/writeRequest";

    const std::string readRequestTopicName = "~/" + this->model->GetName() + "/readRequest";

    const std::string readResponseTopicName = "~/" + this->model->GetName() + "/readResponse";

    // Subscribe to the topic, and register a callback
    this->writeRequestSub = this->node->Subscribe(writeRequestTopicName, &GzSimplePlugin::handleWriteRequest, this);

    this->readRequestSub = this->node->Subscribe(readRequestTopicName, &GzSimplePlugin::handleReadRequest, this);

    gazebo::transport::PublisherPtr pub = node->Advertise<gz_msgs::GzReadResponse>(readResponseTopicName);

    this->pubMap[pub->GetTopic()] = pub;

    int i = 0;
    for(auto joint: joints) {
        joint->SetProvideFeedback(true);
        this->jointNameMap[joint->GetName()] = i++;//TODO: testar ciração do jointNameMap
    }
}

void GzSimplePlugin::handleWriteRequest(GzWriteRequestPtr &msg)
{
    //TODO: tratar exibição de mensagens de Debug como opcional, passado como argumento
    std::cout << "WriteRequest" << std::endl;
    std::cout << std::to_string(msg->jointnames_size()) << std::endl;

    if(msg->jointnames_size() > 0) {

        if(msg->jointnames_size() == msg->pospid_size())
            setPosPids(msg);

        if(msg->jointnames_size() == msg->velpid_size())
            setVelPids(msg);

        if(msg->jointnames_size() == msg->pos_size() && msg->pos_size() == msg->vel_size()) {
            setPositions(msg);
            setVelocities(msg);
        }else if (msg->jointnames_size() == msg->torque_size())
            setTorques(msg);
    }
}

void GzSimplePlugin::handleReadRequest(GzReadRequestPtr &msg) {

    std::cout << "readRequest" << std::endl;

    if(msg->jointnames_size() > 0){

        gz_msgs::GzReadResponse response;

        for(int i = 0; i < msg->jointnames_size(); i++)
            response.add_jointnames(msg->jointnames(i));

        bool posCondition = msg->has_pos() && msg->pos();
        bool velCondition = msg->has_vel() && msg->vel();
        bool torqueCondition = msg->has_torque() && msg->torque();
        bool posVelPidCondition = msg->has_posvelpid() && msg->posvelpid();

        if(posCondition && velCondition && torqueCondition){//situação comum
            getJointStates(&response);
        }else{
            if(posCondition)
                getPositions(&response);
            if(velCondition)
                getVelocities(&response);
            if(torqueCondition)
                getTorques(&response);
        }

        if(posVelPidCondition)
            getPosVelPids(&response);



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


void GzSimplePlugin::setPositions(GzWriteRequestPtr &msg){
    for(int i = 0; i < msg->jointnames_size(); i++){
        this->jointController->SetPositionTarget(this->joints.at(jointNameMap[msg->jointnames(i)])->GetScopedName(),msg->pos(i));
        std::cout << "SetPosition of Joint " << msg->jointnames(i) << ": "<< msg->pos(i) << std::endl;
    }
}

void GzSimplePlugin::setVelocities(GzWriteRequestPtr &msg){
    for(int i = 0; i < msg->jointnames_size(); i++){
        this->jointController->SetVelocityTarget(this->joints.at(jointNameMap[msg->jointnames(i)])->GetScopedName(),msg->vel(i));
        std::cout << "SetVelocity of Joint " << msg->jointnames(i) << ": "<< msg->vel(i) << std::endl;

    }
}

void GzSimplePlugin::setTorques(GzWriteRequestPtr &msg){
    for(int i = 0; i < msg->jointnames_size(); i++){
        //TODO: verificar se esta força está coerente
        this->joints.at(jointNameMap[msg->jointnames(i)])->SetForce(0,msg->torque(i));
        std::cout << "SetTorque of Joint " << msg->jointnames(i) << ": "<< msg->torque(i) << std::endl;

    }
}

std::string gzPidToString(gazebo::common::PID pid) {
    return "P = " + std::to_string(pid.GetPGain()) + "; I = " + std::to_string(pid.GetIGain()) + "; D = " + std::to_string(pid.GetDGain()) + ";";
}

void GzSimplePlugin::setPosPids(GzWriteRequestPtr& msg)
{
    for (int i = 0; i < msg->jointnames_size(); i++) {
        gazebo::common::PID gzPid(msg->pospid(i).kp(), msg->pospid(i).ki(), msg->pospid(i).kd());
        jointController->SetPositionPID(this->joints.at(jointNameMap[msg->jointnames(i)])->GetScopedName(),gzPid);
        std::cout << "SetPositionPID of Joint " << msg->jointnames(i) << ": "<< gzPidToString(gzPid) << std::endl;
    }
}

void GzSimplePlugin::setVelPids(GzWriteRequestPtr& msg)
{
    for (int i = 0; i < msg->jointnames_size(); i++) {
        gazebo::common::PID gzPid(msg->velpid(i).kp(), msg->velpid(i).ki(), msg->velpid(i).kd());
        jointController->SetVelocityPID(this->joints.at(jointNameMap[msg->jointnames(i)])->GetScopedName(),gzPid);
        std::cout << "SetVelocityPID of Joint " << msg->jointnames(i) << ": "<< gzPidToString(gzPid) << std::endl;
    }
}

void GzSimplePlugin::getJointStates(gz_msgs::GzReadResponse *response) {

    for (int i = 0; i < response->jointnames_size(); i++) {
        //TODO: organizar para que não fique código repetido

#if GAZEBO_MAJOR_VERSION >= 8
        response->add_pos(joints[jointNameMap[response->jointnames(i)]]->Position(0));
#else
        response->add_pos(joints[jointNameMap[response->jointnames(i)]]->GetAngle(0).Radian());
#endif

        std::cout << "GetPosition of Joint " << response->jointnames(i) << ": "<< response->pos(i) << std::endl;

        response->add_pos(joints[jointNameMap[response->jointnames(i)]]->GetVelocity(0));

        std::cout << "GetVelocity of Joint " << response->jointnames(i) << ": "<< response->pos(i) << std::endl;

        //gazebo::physics::JointWrench wrench = joints.at(jointNameMap[response->jointnames(i)])->GetForceTorque(0);

#if GAZEBO_MAJOR_VERSION >= 8
        //FIXME: CÁLCULO ERRADO
        response->add_torque(wrench.body1Torque.X()+wrench.body1Torque.Y()+wrench.body1Torque.Z());
        std::cout << std::to_string(wrench.body1Torque.X()) + " " + std::to_string(wrench.body1Torque.Y()) + " " +std::to_string(wrench.body1Torque.Z())<< std::endl;
        std::cout << std::to_string(wrench.body2Torque.X()) + " " + std::to_string(wrench.body2Torque.Y()) + " " +std::to_string(wrench.body2Torque.Z())<< std::endl;
#else
        //TODO: verificar se esta certo
        response->add_torque(joints[jointNameMap[response->jointnames(i)]]->GetForce(0));
#endif
        std::cout << "GetTorque of Joint " << response->jointnames(i) << ": "<< response->torque(i) << std::endl;
    }

}

void GzSimplePlugin::getPositions(gz_msgs::GzReadResponse *response)
{
    for (int i = 0; i < response->jointnames_size(); i++) {

#if GAZEBO_MAJOR_VERSION >= 8
        response->add_pos(joints[jointNameMap[response->jointnames(i)]]->Position(0));
#else
        response->add_pos(joints[jointNameMap[response->jointnames(i)]]->GetAngle(0).Radian());
#endif
        std::cout << "GetPosition of Joint " << response->jointnames(i) << ": "<< response->pos(i) << std::endl;

    }
}

void GzSimplePlugin::getVelocities(gz_msgs::GzReadResponse *response)
{
    for (int i = 0; i < response->jointnames_size(); i++) {

        response->add_pos(joints[jointNameMap[response->jointnames(i)]]->GetVelocity(0));

        std::cout << "GetVelocity of Joint " << response->jointnames(i) << ": "<< response->pos(i) << std::endl;

    }
}

void GzSimplePlugin::getTorques(gz_msgs::GzReadResponse *response)
{
    for (int i = 0; i < response->jointnames_size(); i++) {

        //gazebo::physics::JointWrench wrench = joints.at(jointNameMap[response->jointnames(i)])->GetForceTorque(0);

#if GAZEBO_MAJOR_VERSION >= 8
        //FIXME: CÁLCULO ERRADO
        response->add_torque(wrench.body1Torque.X()+wrench.body1Torque.Y()+wrench.body1Torque.Z());

        std::cout << std::to_string(wrench.body1Torque.X()) + " " + std::to_string(wrench.body1Torque.Y()) + " " +std::to_string(wrench.body1Torque.Z())<< std::endl;
        std::cout << std::to_string(wrench.body2Torque.X()) + " " + std::to_string(wrench.body2Torque.Y()) + " " +std::to_string(wrench.body2Torque.Z())<< std::endl;
#else
        //TODO: verificar se esta certo
        response->add_torque(joints[jointNameMap[response->jointnames(i)]]->GetForce(0));
#endif
        std::cout << "GetTorque of Joint " << response->jointnames(i) << ": "<< response->torque(i) << std::endl;

    }
}

void GzSimplePlugin::getPosVelPids(gz_msgs::GzReadResponse *response){

}
}
