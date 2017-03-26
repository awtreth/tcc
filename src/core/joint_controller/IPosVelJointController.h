#ifndef IPOSVEL_JOINTCONTROLLER_H
#define IPOSVEL_JOINTCONTROLLER_H

#include <vector>
#include <PosVelWriteJointCommand.h>

class IPosVelJointController {

    public:

    virtual bool sendCommand(std::vector<PosVelWriteJointCommand> cmd);

};

typedef std::shared_ptr<IPosVelJointController> PosVelJointControllerPtr;

#endif
