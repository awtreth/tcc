#ifndef JOINT_COMMAND_H
#define JOINT_COMMAND_H

#include <string>
#include <memory>


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

    //FIXME: métodos do tipo has*() podem gerar problemas
    virtual bool hasPosVel(){ return false;}

    virtual bool hasTorque(){ return false; }

    virtual bool hasPosVelPid(){ return false; }

    std::string getJointName() const{return jointName;}

    void setJointName(const std::string &value){jointName = value;}
};

typedef std::shared_ptr<JointCommand> JointCommandPtr;

#endif
