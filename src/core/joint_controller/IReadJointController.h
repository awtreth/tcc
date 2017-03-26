#ifndef IREAD_JOINT_CONTROLLER_H
#define IREAD_JOINT_CONTROLLER_H

#include <vector>
#include <string>
#include <Joint.h>
#include <memory>
#include <map>
#include <ReadJointCommand.h>

class IReadJointController{

    virtual bool sendRequest() = 0;

    virtual bool sendRequest(std::vector<ReadJointCommand> cmds) = 0;

//    virtual bool addRequest(std::vector<ReadJointCommand> cmds) = 0;

    virtual bool addRequest(ReadJointCommand cmd) = 0;

    virtual JointVec getLastJointState() = 0;

};

typedef std::shared_ptr<IReadJointController> ReadJointControllerPtr;

#endif
