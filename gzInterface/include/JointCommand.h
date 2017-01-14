#ifndef JOINT_COMMAND_H
#define JOINT_COMMAND_H

#include <string>
#include <vector>
#include <map>
#include <memory>

/**
 * @brief
 *
 */
class JointCommand {

    public:

    enum Type{READ,WRITE,UNDEFINED};

    protected:

    Type type;

    virtual bool hasParam(const char* param){ return false; }

    Type getType() const { return type;}

};

class PosVelJointCommand {
    private:

    double pos;
    double vel;

    public:

    PosVelJointCommand(double _pos, double _vel) : pos(_pos), vel(_vel){

    }

};

class PosVelJointCommand {

};


typedef std::shared_ptr<AbsJointCommand> JointCommandPtr;



#endif
