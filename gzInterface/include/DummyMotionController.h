#ifndef DUMMY_MOTION_CONTROLLER_H
#define DUMMY_MOTION_CONTROLLER_H

#include <MotionController.h>
#include <AsyncLogger.h>

class DummyMotionController:public MotionController {

private:
    AsyncLogger logger;
    Pose pose;

    // MotionController interface
protected:

    void writeCmd(std::vector<WriteJointCommandPtr> cmd){
        logger.write("WRITE " + pose.toString());
    }

    std::vector<Joint> readCmd(std::vector<ReadJointCommandPtr> cmd){
        logger.write("READ");
    }

    std::vector<WriteJointCommandPtr> onWriteCmd(Pose currentPose){
        pose = currentPose;
        return MotionController::onWriteCmd(currentPose);
    }
};


#endif
