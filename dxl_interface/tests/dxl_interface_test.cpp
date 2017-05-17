#include <iostream>
#include <dxl_model_spec.h>
#include <yaml-cpp/yaml.h>

using namespace dynamixel;
using namespace std;

int main(){

    ModelSpec spec = ModelSpec::getByNumber(311,"../model_specs");

    std::cout << spec.toString() << std::endl;

    return 0;
}
