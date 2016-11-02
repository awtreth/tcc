#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gz_interface_msgs.pb.h>
#include <gazebo/gazebo_client.hh>

#include "DxlMemMap.h"
#include <iostream>
#include <jsoncpp/json/json.h>
#include "DxlModelParamConverter.h"
#include <string>

// for convenience
//using json = nlohmann::json;

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{


	//DxlMemMap map("../json/MX-106.dxlmap");

	//std::cout << map.toString();

	DxlModelParamConverter parser("../json/MX-106.dxlparam");

	std::cout << "Model: " + std::to_string(parser.getModel()) << std::endl;
	std::cout << "PosRes: " + std::to_string(parser.getPosResolution()) << std::endl;
	std::cout << "VelRes: " + std::to_string(parser.getVelResolution()) << std::endl;
	std::cout << "TorRes: " + std::to_string(parser.getTorqueResolution()) << std::endl;
	std::cout << "aRange: " + std::to_string(parser.getAngleRange()) << std::endl;



}
