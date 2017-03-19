#include <PosVelMotionController.h>



PosVelMotionController::PosVelMotionController(PosVelJointControllerPtr _jointController):AbsMotionController()
{
this->jointController = _jointController;
}

void PosVelMotionController::read()
{
}

void PosVelMotionController::afterRead()
{
}

void PosVelMotionController::onWrite()
{
}

void PosVelMotionController::write()
{
}
