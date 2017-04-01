#ifndef TORQUE_WRITE_JOINT_COMMAND_H
#define TORQUE_WRITE_JOINT_COMMAND_H

#include <WriteJointCommand.h>

class TorqueWriteJointCommand : public WriteJointCommand {

    private:

    double torque;

    public:

    static constexpr const char * CMD_ID = "TORQUE_WRITE_JOINT_COMMAND";

    TorqueWriteJointCommand(){
        cmdID = CMD_ID;
    }

    TorqueWriteJointCommand(std::string _jointName, double _torque) : TorqueWriteJointCommand(){
        jointName = _jointName;
        torque = _torque;
    }

    double getTorque() const{return torque;}

    void setTorque(double value){torque = value;}

    //FIXME: métodos do tipo has*() podem gerar problemas
    bool hasTorque(){return true;}
};

//typedef std::shared_ptr<TorqueWriteJointCommand> TorqueWriteJointCommandPtr;

#endif
