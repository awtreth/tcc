#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//#include <gz_interface_msgs.pb.h>
#include <gazebo/gazebo_client.hh>

#include "DxlMemMap.h"
#include <iostream>
#include <jsoncpp/json/json.h>
#include "DxlModelParamConverter.h"
#include <Joint.h>
#include <JointController.h>
#include <GzJointController.h>

#include <memory>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include "gnuplot-iostream.h"

// for convenience
//using json = nlohmann::json;

void func (const gazebo::transport::ConnectionPtr& c) {
    std::cout << "recebeu" << std::endl;
    std::cout << c->GetId() << std::endl;
    std::cout << c->GetIPWhiteList() << std::endl;
    std::cout << c->GetLocalAddress() << std::endl;
    std::cout << c->GetLocalPort() << std::endl;
    std::cout << c->GetLocalURI() << std::endl;
    std::cout << c->GetRemoteAddress() << std::endl;
    std::cout << c->GetRemoteHostname() << std::endl;
    std::cout << c->GetRemotePort() << std::endl;
    std::cout << c->GetRemoteURI() << std::endl;
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

    Gnuplot gp;


    std::vector<std::string> vecStr;

    vecStr.push_back("Motor 1");
    vecStr.push_back("Motor 2");

    std::cout << "criando controller" << std::endl;

    GzJointController controller(vecStr,"~/simple_arm/writeRequest", "~/simple_arm/readRequest", "~/simple_arm/readResponse");

    std::cout << "conexão estabelecida" << std::endl;

    PidValues pid(0.1,0,0);

    std::cout << "write PID " + std::to_string(pid.kp) << std::endl;

    std::vector<PidValues> pidVec(2,pid);


    controller.setPosVelPid(pidVec,pidVec);

    std::cout << "PID enviado" << std::endl;

    std::cout << "writePosVel" << std::endl;


    std::vector<JointCommandPtr> cmdVec;

    cmdVec.push_back(std::make_shared<JointPosVelCommand>(JointPosVelCommand(45*3.141592/180,0)));
    cmdVec.push_back(std::make_shared<JointPosVelCommand>(JointPosVelCommand(-15*3.141592/180,0)));

    controller.sendCommand(cmdVec);

//    std::vector<JointPosVelCommand> posVelCmdVec;

//    posVelCmdVec.push_back(JointPosVelCommand(45*3.141592/180,0));
//    posVelCmdVec.push_back(JointPosVelCommand(-15*3.141592/180,0));

//    controller.goPosVel(posVelCmdVec);

    std::cout << "PosVel enviado" << std::endl;

    //getchar();

    std::ofstream ofs("/tmp/gzLog.txt");

    while(1) {
        std::cout << "read JointState" << std::endl;

        controller.readJointStates();

        JointState jState = controller.getLastJointState(0);

        ofs << jState.position << " " << jState.velocity << std::endl;

        usleep(100000);
    }

    ofs.close();


    //JointPosVelCommand jcmd(2,3);

    //jcmd.hasPosVel();

    //JointPosVelCommand jcmd(2,3);

}
