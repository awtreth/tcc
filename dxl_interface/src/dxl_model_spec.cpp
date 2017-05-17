#include <dxl_model_spec.h>

#include <yaml-cpp/yaml.h>
#include <dirent.h>
#include <cstring>

#include <iostream>

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

dynamixel::ModelSpec::ModelSpec(const char *fileName){
    YAML::Node model = YAML::LoadFile(fileName);

    assert(model["name"]);
    assert(model["number"]);

    assert(model["valueToPositionRatio"]);
    assert(model["valueToVelocityRatio"]);

    assert(model["zeroPositionValue"]);
    assert(model["zeroVelocityValue"]);

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

    valueToPositionRatio = model["valueToPositionRatio"].as<double>();
    valueToVelocityRatio = model["valueToVelocityRatio"].as<double>();

    zeroPositionValue = model["zeroPositionValue"].as<int>();
    zeroVelocityValue = model["zeroVelocityValue"].as<int>();

}

dynamixel::ModelSpec::ModelSpec(const std::vector<std::string> _names, const std::vector<int> _numbers, const double valueToPosRatio, const double valueToVelRatio, const int zeroPosValue, const int zeroVelValue){
    names = _names;
    numbers = _numbers;
    valueToPositionRatio = valueToPosRatio;
    valueToVelocityRatio = valueToVelRatio;
    zeroPositionValue = zeroPosValue;
    zeroVelocityValue = zeroVelValue;
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
        std::cout << fileName << std::endl;

        ModelSpec model(fileName.c_str());

        for(auto name : model.names){
            if(name == modelName)
                return model;
        }
    }

    return ModelSpec();
}

double dynamixel::ModelSpec::valueToRadian(int posValue, bool wheelModeOrRead){
    if(wheelModeOrRead)
        return (posValue-zeroPositionValue)*valueToPositionRatio;
    else
        return posValue*valueToPositionRatio;
}

double dynamixel::ModelSpec::valueToVelocity(int velValue){
    return (velValue-zeroVelocityValue)*valueToVelocityRatio;
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

    str += "name:\n";

    for(auto name : names)
        str += "  - " + name + "\n";

    str += "number:\n";

    for(auto number : numbers)
        str += "  - " + std::to_string(number) + "\n";

    str += "valueToPositionRatio: " + std::to_string(valueToPositionRatio) + "\n";
    str += "valueToVelocityRatio: " + std::to_string(valueToVelocityRatio) + "\n";

    str += "zeroPositionValue: " + std::to_string(zeroPositionValue) + "\n";
    str += "zeroVelocityValue: " + std::to_string(zeroVelocityValue) + "\n";

    return str;
}
