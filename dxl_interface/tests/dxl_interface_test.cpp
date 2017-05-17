#include <iostream>
#include <dxl_model_spec.h>
#include <yaml-cpp/yaml.h>

using namespace dynamixel;
using namespace std;

int main(){

    ModelSpec spec = ModelSpec::getByNumber(18,"../model_specs");

    std::cout << spec.toString() << std::endl;

    std::cout << spec.hasName("AX-12+") << std::endl;
    std::cout << spec.hasNameLike("AX") << std::endl;
    std::cout << spec.hasNumber(13) << std::endl;
    std::cout << spec.hasNameLike("18") << std::endl;
    std::cout << spec.hasName("AX-1") << std::endl;


    return 0;
}
