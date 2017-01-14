#ifndef POS_VEL_MOTION_CONTROLLER_H
#define POS_VEL_MOTION_CONTROLLER_H

#include <AbsMotionController.h>
#include <JointController.h>
#include <iostream>
#include <memory>

class PosVelMotionController : public AbsMotionController {


    private:

    PosVelJointControllerPtr jointController;


    public:

    PosVelMotionController(PosVelJointControllerPtr _jointController);


    // AbsMotionController interface
    protected:
    virtual void read();

    virtual void afterRead();

    virtual void onWrite();

    virtual void write();
};




#endif
