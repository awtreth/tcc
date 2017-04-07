#include<PosVelMotionController.h>


PosVelMotionController::PosVelMotionController(){
    
}

PosVelMotionController::PosVelMotionController(IPosVelJointController *wjc, IReadJointController *rjc){
    
}

std::vector<PosVelWriteJointCommand> PosVelMotionController::onWriteCmd(Pose currentPose){
    return currentPose.toJointCommand();
}
