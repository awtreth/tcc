#ifndef IPOSVEL_JOINTCONTROLLER_H
#define IPOSVEL_JOINTCONTROLLER_H

#include <vector>
#include <PosVelWriteJointCommand.h>

class IPosVelJointController {

    public:

    virtual bool sendCommand(std::vector<PosVelWriteJointCommand> cmd) = 0;

    //virtual bool sendPosVelCommand() = 0;

//    virtual bool addTorqueCommand(std::vector<PosVelWriteJointCommand> cmds) = 0;

    virtual bool addCommand(PosVelWriteJointCommand cmd) = 0;

};

typedef std::shared_ptr<IPosVelJointController> PosVelJointControllerPtr;

#endif
