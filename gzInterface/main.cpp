#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gz_interface_msgs.pb.h>
#include <gazebo/gazebo_client.hh>

#include "DxlMemMap.h"
#include <iostream>
//#include "json.hpp"
#include <jsoncpp/json/json.h>


// for convenience
//using json = nlohmann::json;

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{


	DxlMemMap map("../json/MX-106.dxlmap");

	std::cout << map.toString();

}
