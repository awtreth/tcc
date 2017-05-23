#include <iostream>
#include <dxl_interface.h>
#include <unistd.h>

using namespace dxl_interface;
using namespace std;

int main(){

    auto spec = ModelSpec::getByName("AX-12","../model_specs/");

    std::cout << spec.toString() << std::endl;

//    DxlChannel channel;

//    channel.scan(1,1);
//    channel.scan(3,1);

//    auto handleGroup = channel.getHandleGroup();

//    DxlHandle& handle1 = handleGroup.get(1);
//    DxlHandle& handle3 = handleGroup.get(3);

//    handle1.setPositionVelocity(1.2,1);
//    handle3.setPositionVelocity(2.0,1);

//    WriteCommand cmd(INST_SYNC_WRITE,1);

//    for(DxlHandle& handle : handleGroup.getHandles()){
//        for(auto cmdUnit : handle.getWriteCommandUnits())
//            cmd.addCommandUnit(cmdUnit);
//    }

//    channel.write(cmd);

////    sleep(3);

//    handle1.requestPosition();
//    handle1.requestVelocity();

//    handle3.requestPosition();
//    handle3.requestVelocity();

//    ReadCommand cmdRead(INST_READ,1);

//    for(DxlHandle& handle : handleGroup.getHandles()){
//        for(auto cmdUnit : handle.getReadCommandUnits())
//            cmdRead.addCommandUnit(cmdUnit);
//    }

//    channel.read(cmdRead);

//    std::cout << handle1.getPosition() << std::endl;
//    std::cout << handle1.getVelocity() << std::endl;
//    std::cout << handle3.getPosition() << std::endl;
//    std::cout << handle3.getVelocity() << std::endl;

    return 0;
}
