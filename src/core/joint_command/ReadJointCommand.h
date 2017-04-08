#ifndef READ_JOINT_COMMAND_H
#define READ_JOINT_COMMAND_H

#include <JointCommand.h>

class ReadJointCommand : public JointCommand {

private:
    bool pos = true;
    bool vel = true;
    bool torque = true;
    //    bool posPid = false;
    //    bool velPid = false;


public:

    const char* CMD_ID = "READ_JOINT_COMMAND";

    ReadJointCommand(){
        type = JointCommand::Type::READ;
        cmdID = CMD_ID;
    }

    bool hasPos(){return pos;}
    bool hasVel(){return vel;}
    bool hasTorque(){return torque;}
    bool hasPosVel(){return pos&&vel;}
    bool hasJointState(){return pos&&vel&&torque;}

    void addPos(){pos = true;}
    void rmPos(){pos = false;}

    void addVel(){vel = true;}
    void rmVel(){vel = false;}

    void addTorque(){torque = true;}
    void rmTorque(){torque = false;}

    void addPosVel(){addPos();addVel();}
    void rmPosVel(){rmPos();rmVel();}

    void addJointState(){addPos();addVel();addTorque();}
    void rmJointState(){rmPos();rmVel();rmTorque();}
};

//typedef std::shared_ptr<ReadJointCommand> ReadJointCommandPtr;

#endif
