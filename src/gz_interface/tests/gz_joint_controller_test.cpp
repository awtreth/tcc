#include <GzJointController.h>
#include <vector>
#include <string>
#include <PosVelWriteJointCommand.h>
#include <TorqueWriteJointCommand.h>
#include <iostream>

int main(){

    std::vector<std::string> jointVec;

    jointVec.push_back("arm_sholder_pan_joint");
    jointVec.push_back("arm_elbow_pan_joint");

    GzJointController cont(jointVec,"~/simple_arm/writeRequest", "~/simple_arm/readRequest","~/simple_arm/readResponse");

    std::vector<PosVelWriteJointCommand> writeCmd;
    writeCmd.push_back(PosVelWriteJointCommand(jointVec[0],-1,0));
    writeCmd.push_back(PosVelWriteJointCommand(jointVec[1],2,0));

    cont.sendCommand(writeCmd);

    std::vector<ReadJointCommand> readCmd;
    readCmd.push_back(ReadJointCommand(jointVec[0]));
    readCmd.push_back(ReadJointCommand(jointVec[1]));


    while(1){
        cont.sendRequest(readCmd);

        auto result = cont.getLastJointState();

        for(auto joint : result)
            std::cout << joint.second.getJointState().toString() << std::endl;

        std::cout << std::endl;

        sleep(1);
    }



    return 0;
}
