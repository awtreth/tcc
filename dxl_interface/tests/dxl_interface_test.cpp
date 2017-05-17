#include <iostream>
#include <dxl_model_spec.h>
#include <dxl_read_command.h>
#include <dynamixel_sdk.h>

using namespace dynamixel;
using namespace std;

int main(){

    std::vector<int> v1(2);
    std::vector<int> v2(2);
    std::vector<int> v3(2);
    std::vector<uint8_t*> v4(2);

    try{
        DxlWriteCommand cmd(INST_BULK_WRITE,2,v1,v2,v3,v4);
    }catch(std::exception& e){
        std::cout << e.what() << std::endl;
    }

    return 0;
}
