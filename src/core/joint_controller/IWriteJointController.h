#ifndef IWRITE_JOINT_CONTROLLER_H
#define IWRITE_JOINT_CONTROLLER_H

#include <vector>
#include <string>
#include <WriteJointCommand.h>

class IWriteJointController{

    public:

    virtual bool sendCommand(std::vector<WriteJointCommand> cmds) = 0;

    virtual bool loadCommand(std::vector<WriteJointCommand> cmds) = 0;

};

typedef std::shared_ptr<IWriteJointController> WriteJointControllerPtr;

#endif
