#include <dxl_model_spec.h>

#include <yaml-cpp/yaml.h>
#include <dirent.h>
#include <cstring>
#include <algorithm>
#include <string>
#include <cmath>

using namespace dxl_interface;

double ModelSpec::getValueToPositionRatio() const
{
    return valueToPositionRatio;
}

double ModelSpec::getValueToVelocityRatio() const
{
    return valueToVelocityRatio;
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

std::vector<std::__cxx11::string> dxl_interface::ModelSpec::listFiles(const char *folder, const char* file_extension){
    DIR *dir;
    struct dirent *ent;
    dir = opendir (folder);

    assert(dir != NULL);

    std::vector<std::string> list;

    std::string filePath = folder;

    if(filePath.back() != '/')
        filePath.append("/");

    size_t extensionSize = strlen(file_extension);

    while ((ent = readdir (dir)) != NULL) {
        //if the file ends with DEFAULT_MODEL_SPEC_FILE_EXTENSION

        if( strlen(ent->d_name) > extensionSize &&
                std::string(ent->d_name).substr(strlen(ent->d_name)-extensionSize, extensionSize)==file_extension )
            list.push_back(filePath + ent->d_name);
    }

    closedir(dir);

    return list;
}

dxl_interface::ModelSpec::ModelSpec(){}

dxl_interface::ModelSpec::ModelSpec(const char *fileName){
    YAML::Node model = YAML::LoadFile(fileName);

    //ASSERTION STEP

    //model_id
    assert(model["name"] && model["number"] && model["protocol"]);

    //spec
    assert(model["valueToPositionRatio"] && model["valueToVelocityRatio"]);

    //control_table
    YAML::Node control_table = model["control_table"];
    assert(model["control_table_size"] && control_table);
    assert(control_table[MODEL_ITEM_NAME] && control_table[ID_ITEM_NAME]);
    assert(control_table[GOAL_POSITION_ITEM_NAME] && control_table[MOVING_SPEED_ITEM_NAME]);
    assert(control_table[PRESENT_POSITION_ITEM_NAME] && control_table[PRESENT_SPEED_ITEM_NAME]);

    for(auto item : control_table)
        assert(item.second[ADDRESS_CT_ITEM_NAME] && item.second[LENGTH_CT_ITEM_NAME] && item.second[ACCESS_CT_ITEM_NAME]);


    //ASSIGNMENT STEP

    //spec
    if(model["name"].IsSequence()){
        for(auto name : model["name"])
            names.push_back(name.as<std::string>());
    }else
        names.push_back(model["name"].as<std::string>());

    if(model["number"].IsSequence()){
        for(auto number : model["number"])
            numbers.push_back(number.as<int>());
    }else
        numbers.push_back(model["number"].as<int>());

    protocol = model["protocol"].as<float>();

    valueToPositionRatio = model["valueToPositionRatio"].as<double>();
    valueToVelocityRatio = model["valueToVelocityRatio"].as<double>();

    controlTableSize = model["control_table_size"].as<int>();

    //control_table
    for(auto item : control_table){
        ControlTableItem ctItem;

        ctItem.name = item.first.as<std::string>();
        ctItem.address = item.second[ADDRESS_CT_ITEM_NAME].as<int>();
        ctItem.length = item.second[LENGTH_CT_ITEM_NAME].as<int>();
        ctItem.isWritable = (item.second[ACCESS_CT_ITEM_NAME].as<std::string>()=="RW")?true:false;

        ctItems[ctItem.name] = ctItem;
    }
}

bool ModelSpec::isValid()
{
    return valueToPositionRatio <= 0;
}

dxl_interface::ModelSpec dxl_interface::ModelSpec::getByNumber(int modelNumber, const char *folder, const char* file_extension){

    auto fileNames = listFiles(folder, file_extension);

    for(auto fileName : fileNames){
        ModelSpec model(fileName.c_str());

        for(auto number : model.numbers){
            if(number == modelNumber){
                return model;
            }
        }
    }

    return ModelSpec();
}

dxl_interface::ModelSpec dxl_interface::ModelSpec::getByName(const char *modelName, const char *folder, const char* file_extension){
    auto fileNames = listFiles(folder, file_extension);

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

double dxl_interface::ModelSpec::valueToRadian(int posValue){
    return posValue*valueToPositionRatio;
}

double dxl_interface::ModelSpec::valueToVelocity(int velValue){

    if(velValue < 1024)
        return velValue*valueToVelocityRatio;
    else
        return (1024 - velValue)*valueToVelocityRatio;
}

int dxl_interface::ModelSpec::radianToValue(double pos){
    return int(round(pos/valueToPositionRatio));
}

int dxl_interface::ModelSpec::velocityToValue(double vel, bool wheelMode){

    auto baseValue = int(round(vel/valueToVelocityRatio));

    if(wheelMode){
        if(int(protocol) == 1 && vel >= 0)
            return (baseValue+1024)>2047?2047:(baseValue+1024);
        else
            return (-baseValue)>1023?1023:-baseValue;
    }else{
        auto absValue = abs(baseValue);

        if(absValue >= 1024)
            return 1023;
        else if(absValue == 0)
            return 1;
        else
            return absValue;
    }
}

std::vector<std::string> dxl_interface::ModelSpec::getNames() const{return names;}

std::vector<int> dxl_interface::ModelSpec::getNumbers() const{return numbers;}

std::string dxl_interface::ModelSpec::toString(){
    std::string str;

    str += "name:\n";

    for(auto name : names)
        str += "  - " + name + "\n";

    str += "number:\n";

    for(auto number : numbers)
        str += "  - " + std::to_string(number) + "\n";

    str += "valueToPositionRatio: " + std::to_string(valueToPositionRatio) + "\n";
    str += "valueToVelocityRatio: " + std::to_string(valueToVelocityRatio) + "\n";

    str += "control_table_size:" + std::to_string(controlTableSize) + "\n";

    str += "control_table:\n";

    for(auto pair : ctItems){
        auto item = pair.second;
        str += "  " + item.name + ": {"
                + ADDRESS_CT_ITEM_NAME+": "+std::to_string(item.address)+", "
                + ACCESS_CT_ITEM_NAME+": "+((item.isWritable)?("RW"):("R")) + ", "
                + LENGTH_CT_ITEM_NAME+": "+std::to_string(item.length)+"}\n";
    }

    return str;
}
