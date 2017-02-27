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

    virtual bool hasPosVel(){ return false;}

    virtual bool hasTorque(){ return false; }

    virtual bool hasPosVelPid(){ return false; }

    std::string getJointName() const{return jointName;}
    void setJointName(const std::string &value){jointName = value;}
};

typedef std::shared_ptr<JointCommand> JointCommandPtr;//FIXME: JointCommandPtr

class ReadJointCommand : public JointCommand {

    private:
    bool posVel = true;
    bool torque = true;
    bool pid = false;

    public:

    ReadJointCommand(){
        type = JointCommand::Type::READ;
    }

    ReadJointCommand(std::string _jointName){
        jointName = _jointName;
    }

    ReadJointCommand(const char* _cmdID){
        type = JointCommand::Type::READ;
        cmdID = _cmdID;
    }

    ReadJointCommand(const char* _cmdID, bool _posVel, bool _torque, bool _pid){
        type = JointCommand::Type::READ;
        cmdID = _cmdID;

        posVel = _posVel;
        torque = _torque;
        pid = _pid;
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

#define READ_JOINT_STATE_CMD_ID "READ_JOINT_STATE_COMMAND"
#define READ_PSOVEL_PID_STATE_CMD_ID "READ_POSVEL_PID_STATE_COMMAND"

const ReadJointCommand readPosVelTorqueCommand(READ_JOINT_STATE_CMD_ID);
const ReadJointCommand readPosVelPidCommand(READ_PSOVEL_PID_STATE_CMD_ID, false, false, true);


class WriteJointCommand : public JointCommand {

    public:

    const char* CMD_ID = "WRITE_JOINT_COMMAND";

    WriteJointCommand(){
        type = JointCommand::Type::WRITE;
        cmdID = CMD_ID;
    }
};

typedef std::shared_ptr<WriteJointCommand> WriteJointCommandPtr;


class PosVelWriteJointCommand : public WriteJointCommand {

    private:

    double pos;
    double vel;

    public:

    const char* CMD_ID = "POSVEL_WRITE_JOINT_COMMAND";

    PosVelWriteJointCommand(){
        cmdID = CMD_ID;
    }

    PosVelWriteJointCommand(std::string jointName, double _pos, double _vel){
        cmdID = CMD_ID;
        pos = _pos;
        vel = _vel;
    }

    double getPos() const{return pos;}
    void setPos(double value){pos = value;}

    double getVel() const{return vel;}
    void setVel(double value){vel = value;}

    bool hasPosVel(){return true;}
};

typedef std::shared_ptr<PosVelWriteJointCommand> PosVelWriteJointCommandPtr;


class TorqueWriteJointCommand : public WriteJointCommand {

    private:

    double torque;

    public:

    const char * CMD_ID = "TORQUE_WRITE_JOINT_COMMAND";

    TorqueWriteJointCommand(double _torque){
        cmdID = CMD_ID;
        torque = _torque;
    }

    double getTorque() const{return torque;}

    void setTorque(double value){torque = value;}

    bool hasTorque(){return true;}
};

typedef std::shared_ptr<TorqueWriteJointCommand> TorqueWriteJointCommandPtr;


class PidWriteJointCommand : public WriteJointCommand {

};

#endif
