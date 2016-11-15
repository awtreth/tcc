
#include <GzJointController.h>
#include <gazebo/transport/transport.hh>
#include <GzWriteRequest.pb.h>
#include <gazebo/gazebo_client.hh>


GzJointController::GzJointController(std::vector<std::__cxx11::string> jointNames, std::string pubTopic, std::string subTopic) : AbsJointController(jointNames) {

	gazebo::client::setup();//TODO: verificar se é melhor aqui ou fora

	// Create our node for communication
	gazebo::transport::NodePtr pubNode = gazebo::transport::NodePtr(new gazebo::transport::Node());

	pubNode->Init();

	// Publish to the  velodyne topic
	pub = pubNode->Advertise<gz_msgs::WriteRequest>(pubTopic);

	// Wait for a subscriber to connect to this publisher
	//pub->WaitForConnection();

	// Create the node
	gazebo::transport::NodePtr subNode = gazebo::transport::NodePtr(new gazebo::transport::Node());

	subNode->Init();

	// Subscribe to the topic, and register a callback
	sub = subNode->Subscribe(subTopic, &GzJointController::onReadMsg, this);

}



GzJointController::~GzJointController()
{
	gazebo::client::shutdown();//TODO: verificar se é melhor aqui ou fora
}


void GzJointController::onReadMsg(ReadAnswerPtr& msg)
{

}

bool GzJointController::goPosVel(double pos, double vel, int jointID) {
	gz_msgs::WriteRequest msg;

	msg.add_motorid(jointID);

	msg.add_pos(pos);

	msg.add_vel(vel);

	pub->Publish(msg, false);

	return true;
}

bool GzJointController::goPosVel(double pos, double vel, std::__cxx11::string jointName) {
	return true;

}

bool GzJointController::goPosVel(std::vector<JointPosVelCommand> cmd) {
	return true;

}

bool GzJointController::goTorque(double torque, int jointID) {
	return true;

}

bool GzJointController::goTorque(double torque, std::__cxx11::string jointName) {
	return true;

}

bool GzJointController::goTorque(std::vector<JointTorqueCommand>) {
	return true;

}

bool GzJointController::readJointStates() {
	return true;

}
