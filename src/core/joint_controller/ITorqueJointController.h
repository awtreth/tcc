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
class TorqueJointController{

    public:

    virtual bool sendCommand(std::vector<TorqueWriteJointCommand> cmd);

};

typedef std::shared_ptr<TorqueJointController> TorqueJointControllerPtr;


#endif
