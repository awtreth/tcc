#ifndef MYJOINT_H
#define MYJOINT_H

#include <string>
#include <vector>
#include <map>

/**
 * @brief
 *
 */
class AbsJointCommand {

    protected:

    double pos = 0; /**< TODO: describe */
    double vel = 0; /**< TODO: describe */
    double torque = 0; /**< TODO: describe */

    public:

    /**
     * @brief
     *
     * @return bool
     */
    virtual bool hasPosVel() = 0;

    /**
     * @brief
     *
     * @return bool
     */
    virtual bool hasTorque() = 0;

};

/**
 * @brief
 *
 */
class JointPosVelCommand: public AbsJointCommand {

    private:
    /**
     * @brief
     *
     * @return bool
     */
    bool hasPosVel(){return true;}

    /**
     * @brief
     *
     * @return bool
     */
    bool hasTorque(){return false;}

    public:

    /**
     * @brief
     *
     * @param _pos
     * @param _vel
     */
    JointPosVelCommand(double _pos, double _vel) { pos = _pos; vel = _vel;}


    /**
     * @brief
     *
     * @return double
     */
    double getPos(){return pos;}

    /**
     * @brief
     *
     * @return double
     */
    double getVel(){return vel;}
};



/**
 * @brief
 *
 */
class JointTorqueCommand: public AbsJointCommand {

    private:

    /**
     * @brief
     *
     * @return bool
     */
    bool hasPosVel(){return false;}

    /**
     * @brief
     *
     * @return bool
     */
    bool hasTorque(){return true;}

    public:

    /**
     * @brief
     *
     * @param _torque
     */
    JointTorqueCommand(double _torque){ torque = _torque;}


    /**
     * @brief
     *
     * @return double
     */
    double getTorque(){return torque;}

};


/**
 * @brief
 *
 */
class JointState{

    public:

    double position; /**< TODO: describe */

    double velocity; /**< TODO: describe */

    double torque; /**< TODO: describe */

    JointState(): position(0), velocity(0), torque(0) {}

    /**
     * @brief
     *
     * @param pos
     * @param vel
     * @param torq
     */
    JointState(double pos, double vel, double torq) : position(pos), velocity(vel), torque(torq) {}

};

/**
 * @brief
 *
 */
class PidValues {
    public:
    double kp; /**< TODO: describe */
    double ki; /**< TODO: describe */
    double kd; /**< TODO: describe */

    /**
     * @brief
     *
     */
    PidValues():kp(0), ki(0), kd(0) {}

    /**
     * @brief
     *
     * @param p
     * @param i
     * @param d
     */
    PidValues(double p, double i, double d):kp(p),ki(i),kd(d) {}
};


/**
 * @brief
 *
 */
class Joint {

    protected:

    std::string name; /**< TODO: describe */

    JointState jointState; /**< TODO: describe */

    PidValues posPID; /**< TODO: describe */

    PidValues velPID; /**< TODO: describe */

    public:

    /**
     * @brief
     *
     */
    Joint():name(""){}


    /**
     * @brief
     *
     * @param _name
     */
    Joint(std::string _name) {
        name = _name;
    }

    //int getId() { return id; }

    /**
     * @brief
     *
     * @return std::string
     */
    std::string getName() { return name; }

    /**
     * @brief
     *
     * @return JointState
     */
    JointState getJointState() { return jointState; }

    /**
     * @brief
     *
     * @param newJointState
     */
    void setJointState(JointState newJointState) { jointState = newJointState; }

    /**
     * @brief
     *
     * @param kp
     * @param ki
     * @param kd
     */
    void setPosPID(double kp, double ki, double kd){posPID.kp=kp;posPID.ki=ki;posPID.kd=kd;}

    /**
     * @brief
     *
     * @param kp
     * @param ki
     * @param kd
     */
    void setVelPID(double kp, double ki, double kd){velPID.kp=kp;velPID.ki=ki;velPID.kd=kd;}


    /**
     * @brief
     *
     * @param pid
     */
    void setPosPid(PidValues pid) { posPID = pid ; }

    /**
     * @brief
     *
     * @param pid
     */
    void setVelPid(PidValues pid) { velPID = pid;  }


    /**
     * @brief
     *
     * @return PidValues
     */
    PidValues getPosPid() {return posPID;}

    /**
     * @brief
     *
     * @return PidValues
     */
    PidValues getVelPid() {return velPID;}

};

//class PIDJoint : public Joint {
//    private:

//    PID posPID;

//    PID velPID;

//    public:

//    PIDJoint(){ setPosPID(0,0,0); setVelPID(0,0,0); }

//    void setPosPID(double kp, double ki, double kd){posPID.kp=kp;posPID.ki=ki;posPID.kd=kd;}

//    void setVelPID(double kp, double ki, double kd){velPID.kp=kp;velPID.ki=ki;velPID.kd=kd;}

//    PID getPosPID() {return posPID;}

//    PID getVelPID() {return velPID;}

//};


/**
 * @brief
 *
 */
typedef std::vector<Joint> JointVec;

#endif
