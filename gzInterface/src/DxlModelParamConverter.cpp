#include "DxlModelParamConverter.h"
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
	
DxlModelParamConverter::DxlModelParamConverter(){}

DxlModelParamConverter::DxlModelParamConverter(const char *jsonFileName) {
	Json::Value jsonObj;
	std::ifstream jsonFile(jsonFileName);

	jsonFile >> jsonObj;

	this->model = jsonObj["model"].asInt();
	this->posResolution = jsonObj["posResolution"].asDouble();
	this->angleRange = jsonObj["angleRange"].asDouble();
	this->velResolution = jsonObj["velResolution"].asDouble();
	this->torqueResolution = jsonObj["torqueResolution"].asDouble();

}

inline double DxlModelParamConverter::posToRad(short pos) {
	return pos*this->posResolution;
}

inline double DxlModelParamConverter::velToRad(short vel) {
	return vel*this->velResolution;
}

inline double DxlModelParamConverter::torqueToNm(short torque) {
	return torque*this->torqueResolution;
}
