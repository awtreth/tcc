#ifndef ITORQUE_JOINTCONTROLLER_H
#define ITORQUE_JOINTCONTROLLER_H

#include <vector>
#include <string>
#include <Joint.h>
#include <memory>
#include <TorqueWriteJointCommand.h>

/**
 * @brief
 *
 */
class ITorqueJointController{

    public:

    virtual bool sendTorqueCommand(std::vector<TorqueWriteJointCommand> cmd);

    virtual bool sendTorqueCommand() = 0;

//    virtual bool addTorqueCommand(std::vector<TorqueWriteJointCommand> cmds) = 0;

    virtual bool addTorqueCommand(TorqueWriteJointCommand cmd) = 0;

};

typedef std::shared_ptr<ITorqueJointController> TorqueJointControllerPtr;


#endif
