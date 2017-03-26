#ifndef WRITE_JOINT_COMMAND_H
#define WRITE_JOINT_COMMAND_H

#include <JointCommand.h>
#include <memory>

class WriteJointCommand : public JointCommand {

    public:

    const char* CMD_ID = "WRITE_JOINT_COMMAND";

    WriteJointCommand(){
        type = JointCommand::Type::WRITE;
        cmdID = CMD_ID;
    }
};

//typedef std::shared_ptr<WriteJointCommand> WriteJointCommandPtr;

#endif
