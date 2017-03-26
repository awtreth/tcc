#ifndef READ_JOINT_COMMAND_H
#define READ_JOINT_COMMAND_H

#include <JointCommand.h>

class ReadJointCommand : public JointCommand {

    private:
    bool posVel = true;
    bool torque = true;
    bool pid = false;

    public:

    const char* CMD_ID = "READ_JOINT_COMMAND";

    ReadJointCommand(){
        type = JointCommand::Type::READ;
        cmdID = CMD_ID;
    }

    ReadJointCommand(std::string _jointName, bool hasPosVel = true, bool hasTorque = true, bool hasPid = false):ReadJointCommand(){
        jointName = _jointName;
        posVel = hasPosVel;
        torque = hasTorque;
        pid = hasPid;
    }

    bool hasPosVel(){return posVel;}
    bool hasTorque(){return torque;}
    bool hasPosVelPid(){return pid;}

    void addPosVel(){posVel = true;}
    void rmPosVel(){posVel = false;}
    void addTorque(){torque = true;}
    void rmTorque(){torque = false;}
    void addPosVelPid(){pid = true;}
    void rmPosVelPid(){pid = false;}

};

typedef std::shared_ptr<ReadJointCommand> ReadJointCommandPtr;


#endif
