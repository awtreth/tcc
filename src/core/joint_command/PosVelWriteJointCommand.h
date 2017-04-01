#ifndef POSVEL_WRITE_JOINT_COMMAND_H
#define POSVEL_WRITE_JOINT_COMMAND_H

#include <WriteJointCommand.h>

class PosVelWriteJointCommand : public WriteJointCommand {

    private:

    double pos;
    double vel;

    public:

    static constexpr const char* CMD_ID = "POSVEL_WRITE_JOINT_COMMAND";

    PosVelWriteJointCommand(){
        cmdID = CMD_ID;
    }

    PosVelWriteJointCommand(std::string _jointName, double _pos, double _vel):PosVelWriteJointCommand(){
        jointName = _jointName;
        pos = _pos;
        vel = _vel;
    }

    double getPos() const{return pos;}
    void setPos(double value){pos = value;}

    double getVel() const{return vel;}
    void setVel(double value){vel = value;}

    //FIXME: métodos do tipo has*() podem gerar problemas
    bool hasPosVel(){return true;}
};

//typedef std::shared_ptr<PosVelWriteJointCommand> PosVelWriteJointCommandPtr;

#endif
