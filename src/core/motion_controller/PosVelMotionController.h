#ifndef POS_VEL_MOTION_CONTROLLER_H
#define POS_VEL_MOTION_CONTROLLER_H

#include <IPosVelJointController.h>
#include <MotionController.h>
#include <PosVelWriteJointCommand.h>
#include <iostream>
#include <memory>

class PosVelMotionController : public MotionController<IPosVelJointController, PosVelWriteJointCommand> {


    // MotionController interface
protected:

    PosVelMotionController();

    PosVelMotionController(IPosVelJointController* wjc, IReadJointController* rjc);

    virtual std::vector<PosVelWriteJointCommand> onWriteCmd(Pose currentPose);
};




#endif

