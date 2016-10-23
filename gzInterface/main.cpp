#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gz_interface_msgs.pb.h>
#include <gazebo/gazebo_client.hh>

#include "DxlMemMap.h"
#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    DxlMemMap map;

    //std::cout << map.toString();

    char input[] = {10, 20, 4, 3};
    char output[4];


    std::vector<short int> inVec;
    inVec.push_back(257);
    inVec.push_back(514);


    map.set(2,inVec);

    std::vector<short int> outVec = map.getWords(2,2);

    for(int i = 0; i < 2; i++){
        std::cout << outVec[i] << std::endl;
    }

    //std::cout << map.toString(2,4);

    //while(1){
    //    std::cout << map.toString();
    //}
//    gazebo::client::setup(_argc, _argv);

//    // Create our node for communication
//    gazebo::transport::NodePtr node(new gazebo::transport::Node());
//    node->Init();

//    // Publish to the  velodyne topic
//    gazebo::transport::PublisherPtr pub =
//            node->Advertise<gz_interface_msgs::msg::GzSimpleRequest>("~/simple_arm/dxl_cmd");

//    // Wait for a subscriber to connect to this publisher
//    pub->WaitForConnection();

//    // Create a a vector3 message
//    gz_interface_msgs::msg::GzSimpleRequest msg;

//    msg.set_requestitem(gz_interface_msgs::msg::GzSimpleRequest_RequestItem_POS);
//    msg.set_requesttype(gz_interface_msgs::msg::GzSimpleRequest_RequestType_WRITE);
//    msg.add_motorid(0);
//    msg.add_pos(45*3.1415/180.);

//    // Send the message
//    pub->Publish(msg);

//    gazebo::client::shutdown();

}
