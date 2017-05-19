#include <dxl_model_spec.h>

#include <yaml-cpp/yaml.h>
#include <dirent.h>
#include <cstring>
#include <algorithm>
#include <string>

using namespace dynamixel;

double ModelSpec::getValueToPositionRatio() const
{
    return valueToPositionRatio;
}

double ModelSpec::getValueToVelocityRatio() const
{
    return valueToVelocityRatio;
}

int ModelSpec::getZeroPositionValue() const
{
    return zeroPositionValue;
}

int ModelSpec::getZeroVelocityValue() const
{
    return zeroVelocityValue;
}

bool ModelSpec::hasName(const char *name)
{
    return std::find(names.begin(),names.end(),name)!= names.end();
}

bool ModelSpec::hasNumber(const int number)
{
    return std::find(numbers.begin(),numbers.end(),number)!= numbers.end();
}

bool ModelSpec::hasNameLike(const char *str)
{
    return names.end() != std::find_if(names.begin(),names.end(),
                        [str](std::string nm){return nm.find(str)!=std::string::npos;} );
}

int ModelSpec::getControlTableSize() const
{
    return controlTableSize;
}

std::vector<std::__cxx11::string> dynamixel::ModelSpec::listFiles(const char *folder){
    DIR *dir;
    struct dirent *ent;
    dir = opendir (folder);

    assert(dir != NULL);

    std::vector<std::string> list;

    std::string filePath = folder;

    if(filePath.back() != '/')
        filePath.append("/");

    const int extensionSize = strlen(DEFAULT_MODEL_SPEC_FILE_EXTENSION);

    while ((ent = readdir (dir)) != NULL) {
        //if the file ends with DEFAULT_MODEL_SPEC_FILE_EXTENSION

        if( strlen(ent->d_name) > extensionSize &&
                std::string(ent->d_name).substr(strlen(ent->d_name)-extensionSize, extensionSize)==DEFAULT_MODEL_SPEC_FILE_EXTENSION )
            list.push_back(filePath + ent->d_name);
    }

    closedir(dir);

    return list;
}

dynamixel::ModelSpec::ModelSpec(){}
#include <iostream>

dynamixel::ModelSpec::ModelSpec(const char *fileName){
    YAML::Node model = YAML::LoadFile(fileName);

    //ASSERTION STEP
    //1st layer
    YAML::Node spec = model["spec"];
    YAML::Node controlTable = model["control_table"];

    //spec
    assert(spec && controlTable);
    assert(spec["name"]&&spec["number"]);
    assert(spec["valueToPositionRatio"]&&spec["valueToVelocityRatio"]);
    assert(spec["controlTableSize"]);

    //control_table
    assert(controlTable[MODEL_ITEM_NAME]&&controlTable[ID_ITEM_NAME]);
    assert(controlTable[GOAL_POSITION_ITEM_NAME]&&controlTable[MOVING_SPEED_ITEM_NAME]);
    assert(controlTable[PRESENT_POSITION_ITEM_NAME]&&controlTable[PRESENT_SPEED_ITEM_NAME]);

    for(auto item : controlTable){
        assert(item.second[ADDRESS_CT_ITEM_NAME]&&item.second[LENGTH_CT_ITEM_NAME]
               &&item.second[ACCESS_CT_ITEM_NAME]&&item.second[SIGNED_CT_ITEM_NAME]);
    }
    //ASSIGNMENT STEP

    //spec
    if(spec["name"].IsSequence()){
        for(auto name : spec["name"])
            names.push_back(name.as<std::string>());
    }else
        names.push_back(spec["name"].as<std::string>());

    if(spec["number"].IsSequence()){
        for(auto number : spec["number"])
            numbers.push_back(number.as<int>());
    }else
        numbers.push_back(spec["number"].as<int>());

    valueToPositionRatio = spec["valueToPositionRatio"].as<double>();
    valueToVelocityRatio = spec["valueToVelocityRatio"].as<double>();

    controlTableSize = spec["controlTableSize"].as<int>();

    if(spec["zeroPositionValue"])
        zeroPositionValue = spec["zeroPositionValue"].as<int>();
    if(spec["zeroVelocityValue"])
        zeroVelocityValue = spec["zeroVelocityValue"].as<int>();

    //control_table
    for(auto item : controlTable){
        ControlTableItem ctItem;

        ctItem.name = item.first.as<std::string>();
        ctItem.address = item.second[ADDRESS_CT_ITEM_NAME].as<int>();
        ctItem.length = item.second[LENGTH_CT_ITEM_NAME].as<int>();
        ctItem.isWritable = (item.second[ACCESS_CT_ITEM_NAME].as<std::string>()=="RW")?true:false;
        ctItem.isSigned = item.second[SIGNED_CT_ITEM_NAME].as<bool>();

        ctItems[ctItem.name] = ctItem;
    }
}

dynamixel::ModelSpec dynamixel::ModelSpec::getByNumber(int modelNumber, const char *folder){

    auto fileNames = listFiles(folder);

    for(auto fileName : fileNames){
        ModelSpec model(fileName.c_str());

        for(auto number : model.numbers){
            if(number == modelNumber)
                return model;
        }
    }

    return ModelSpec();
}

dynamixel::ModelSpec dynamixel::ModelSpec::getByName(const char *modelName, const char *folder){
    auto fileNames = listFiles(folder);

    for(auto fileName : fileNames){
        ModelSpec model(fileName.c_str());

        for(auto name : model.names){
            if(name == modelName)
                return model;
        }
    }

    return ModelSpec();
}

ControlTableItem ModelSpec::getControlTableItem(const char *name)
{
    return ctItems[name];
}

double dynamixel::ModelSpec::valueToRadian(int posValue, bool wheelModeOrRead){
    if(wheelModeOrRead)
        return (posValue-zeroPositionValue)*valueToPositionRatio;
    else
        return posValue*valueToPositionRatio;
}

double dynamixel::ModelSpec::valueToVelocity(int velValue){
    //FIXME: valido somente para 1.0
    if(velValue>zeroVelocityValue)
        return (velValue-zeroVelocityValue)*valueToVelocityRatio;
    else
        return -velValue*valueToVelocityRatio;
}

int dynamixel::ModelSpec::radianToValue(double pos){
    return int(pos/valueToPositionRatio+zeroPositionValue);
}

int dynamixel::ModelSpec::velocityToValue(double vel, bool wheelMode){
    if(wheelMode)
        return int(vel/valueToVelocityRatio+zeroVelocityValue);
    else
        return int(vel/valueToVelocityRatio);
}

std::vector<std::string> dynamixel::ModelSpec::getNames() const{return names;}

std::vector<int> dynamixel::ModelSpec::getNumbers() const{return numbers;}

std::string dynamixel::ModelSpec::toString(){
    std::string str;

    str += "spec\n";
    str += "  name:\n";

    for(auto name : names)
        str += "    - " + name + "\n";

    str += "  number:\n";

    for(auto number : numbers)
        str += "    - " + std::to_string(number) + "\n";

    str += "  valueToPositionRatio: " + std::to_string(valueToPositionRatio) + "\n";
    str += "  valueToVelocityRatio: " + std::to_string(valueToVelocityRatio) + "\n";

    str += "  zeroPositionValue: " + std::to_string(zeroPositionValue) + "\n";
    str += "  zeroVelocityValue: " + std::to_string(zeroVelocityValue) + "\n";

    str += "control_table:\n";

    for(auto pair : ctItems){
        auto item = pair.second;
        str += "  " + item.name + ": {"
                + ADDRESS_CT_ITEM_NAME+": "+std::to_string(item.address)+", "
                + ACCESS_CT_ITEM_NAME+": "+((item.isWritable)?("RW"):("R")) + ", "
                + LENGTH_CT_ITEM_NAME+": "+std::to_string(item.length)+", "
                + SIGNED_CT_ITEM_NAME+": "+((item.isSigned)?("true"):("false"))+"}\n";
    }

    return str;
}
