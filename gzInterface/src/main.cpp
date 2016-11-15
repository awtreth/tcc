#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//#include <gz_interface_msgs.pb.h>
#include <gazebo/gazebo_client.hh>

#include "DxlMemMap.h"
#include <iostream>
#include <jsoncpp/json/json.h>
#include "DxlModelParamConverter.h"
#include <string>
#include <Joint.h>
#include <JointController.h>

#include <memory>

// for convenience
//using json = nlohmann::json;

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

	JointPosVelCommand jcmd(2,3);

	//jcmd.hasPosVel();

	//JointPosVelCommand jcmd(2,3);

}
