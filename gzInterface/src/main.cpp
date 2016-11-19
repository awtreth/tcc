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

// for convenience
//using json = nlohmann::json;

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

	std::vector<std::string> vecStr;

	vecStr.push_back("Motor 1");
	vecStr.push_back("Motor 2");

    std::cout << "criando controller" << std::endl;

    GzJointController controller(vecStr,"~/simple_arm/writeRequest", "~/output");

    std::cout << "conexão estabelecida" << std::endl;

    PidValues pid(0.1,0,0);

    std::cout << "write PID " + std::to_string(pid.kp) << std::endl;

    controller.setPosVelPid(pid,pid,0);

    std::cout << "PID enviado" << std::endl;

    getchar();
	//JointPosVelCommand jcmd(2,3);

	//jcmd.hasPosVel();

	//JointPosVelCommand jcmd(2,3);

}
