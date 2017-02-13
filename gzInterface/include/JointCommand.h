#ifndef JOINT_COMMAND_H
#define JOINT_COMMAND_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <MapVec.h>

/**
 * @brief
 *
 */
class JointCommand {

    public: enum Type{READ, WRITE, UNDEFINED};

    protected:

    Type type;

    const char* cmdID;

    std::string jointName;

    public:

    JointCommand() {
        type = UNDEFINED;
    }

    JointCommand(const Type _type, const char* _cmdID) {
        type = _type;
        cmdID = _cmdID;
    }

    virtual Type getType() const{return type;}
    virtual const char* getCmdID() const{ return cmdID; }
};



class ReadJointCommand : public JointCommand {

    public:

    ReadJointCommand(const char* _cmdID){
        type = JointCommand::Type::READ;
        cmdID = _cmdID;
    }


};

#define READ_JOINT_STATE_CMD_ID "READ_JOINT_STATE_COMMAND"

const ReadJointCommand readJointStateCommand(READ_JOINT_STATE_CMD_ID);


class WriteJointCommand : public JointCommand {

    public:

    WriteJointCommand(){
        type = JointCommand::Type::WRITE;
    }
};

class PosVelWriteJointCommand : public WriteJointCommand {

    private:

    double pos;
    double vel;

    public:

    static const char * CMD_ID;

    PosVelWriteJointCommand(){
        cmdID = CMD_ID;
    }

    PosVelWriteJointCommand(double _pos, double _vel){
        cmdID = CMD_ID;
        pos = _pos;
        vel = _vel;
    }

    double getPos() const{return pos;}
    void setPos(double value){pos = value;}

    double getVel() const{return vel;}
    void setVel(double value){vel = value;}

};

const char* PosVelWriteJointCommand::CMD_ID = "POSVEL_WRITE_JOINT_COMMAND";


class TorqueWriteJointCommand : public WriteJointCommand {

    private:

    double torque;

    public:

    static const char * CMD_ID;

    TorqueWriteJointCommand(double _torque){
        cmdID = CMD_ID;
        torque = _torque;
    }

    double getTorque() const{return torque;}

    void setTorque(double value){torque = value;}

};

const char* TorqueWriteJointCommand::CMD_ID = "TORQUE_WRITE_JOINT_COMMAND";

#endif

