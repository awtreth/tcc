#ifndef MOTION_CONTROLLER_TIME_HANDLER_H
#define MOTION_CONTROLLER_TIME_HANDLER_H

#include <ControllerTimeHandler.h>
#include <IController.h>
#include <IHardwareInterface.h>

class MotionControllerTimeHandler:public ControllerTimeHandler<IController,IHardwareInterface>{

};

#include <src/ControllerTimeHandler.cpp>



#endif
