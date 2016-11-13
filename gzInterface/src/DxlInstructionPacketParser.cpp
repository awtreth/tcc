#include "DxlInstructionPacketParser.h"
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
#include "DxlMemMapConstants.h"
#include <string>

DxlInstructionPacketParser::DxlInstructionPacketParser(){
	this->basicInstructionType = BasicInstructionType::NONE;
}

bool DxlInstructionPacketParser::loadDxlInstructionPacket(gz_msgs::DxlInstructionPacket _packet) {
	this->packet = _packet;

	if(this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_WRITE_DATA ||
	   this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_SYNC_WRITE ||
	   this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_REG_WRITE) {

		this->basicInstructionType = BasicInstructionType::WRITE;

	}else if(this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_READ_DATA ||
			 this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_BULK_READ){

		this->basicInstructionType = BasicInstructionType::READ;

	}else if(this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_PING ||
			 this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_ACTION ||
			 this->getInstructionType()==gz_msgs::DxlInstructionPacket_InstructionType_RESET){

		this->basicInstructionType = BasicInstructionType::OTHER;

	}else {

		this->basicInstructionType = BasicInstructionType::NONE;
		return false;
	}
	return true;
}

DxlInstructionPacketParser::DxlInstructionPacketParser(gz_msgs::DxlInstructionPacket _packet) {
	this->loadDxlInstructionPacket(_packet);
}


//bool DxlInstructionPacketParser::hasParameter(int addr, int size) {
//	if(this->packet.has_startaddress()) {
//		return (this->packet.length() - addr - this->packet.startaddress()) > size;
//	}
//}

//bool DxlInstructionPacketParser::hasPos() {

//	if(this->packet.has_startaddress()) {

//		if(this->basicInstructionType == BasicInstructionType::WRITE) {
//			return this->hasParameter(P_GOAL_POSITION_L, 2);
//		}else if(this->basicInstructionType == BasicInstructionType::READ) {
//			return this->hasParameter(P_PRESENT_POSITION_L, 2);
//		}
//	}

//	return false;
//}

//bool DxlInstructionPacketParser::hasVel() {

//	if(this->packet.has_startaddress()) {

//		if(this->basicInstructionType == BasicInstructionType::WRITE) {
//			return this->hasParameter(P_GOAL_SPEED_L, 2);
//		}else if(this->basicInstructionType == BasicInstructionType::READ) {
//			return this->hasParameter(P_PRESENT_SPEED_L, 2);
//		}
//	}

//	return false;
//}

//bool DxlInstructionPacketParser::hasTorque() {

//	if(this->packet.has_startaddress()) {

//		if(this->basicInstructionType == BasicInstructionType::WRITE) {
//			return this->hasParameter(P_GOAL_TORQUE_L, 2);
//		}else if(this->basicInstructionType == BasicInstructionType::READ) {
//			return this->hasParameter(P_PRESENT_LOAD_L, 2);
//		}
//	}

//	return false;
//}

////função auxiliar para converter string de 2 caracteres (2 bytes da mensagem) em short
//short str2short(const::std::string str){
//	short output = 0;
//	char* startAddr = str.c_str();

//	std::copy(startAddr, startAddr+2, &output);

//	return output;
//}



//gz_msgs::DxlInstructionPacket::InstructionType DxlInstructionPacketParser::getInstructionType() {
//	return this->packet.instruction();
//}

//short DxlInstructionPacketParser::getPos(int id){


//	int paramPos = this->packet.ids().find((char)id)*this->packet.length() + P_GOAL_POSITION_L - packet.startaddress();

//	return str2short(packet.parameters().substr(paramPos,2));

//}

