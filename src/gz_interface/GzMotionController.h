#include <MotionController.h>
#include <GzJointController.h>
#include <IPosVelJointController.h>
#include <joint_controller/Joint.h>
#include <iostream>

class GzMotionController : public PosVelMotionController {


    // MotionController interface
protected:
    void afterRead(JointMap updatedJoints){
        for(auto item : updatedJoints)
            std::cout << item.second.getJointState().toString() << std::endl;
    }

public:
    GzMotionController():PosVelMotionController(){

    }

    GzMotionController(GzJointController* gzController):PosVelMotionController((IPosVelJointController*)gzController, (IReadJointController*)gzController){

    }

};
