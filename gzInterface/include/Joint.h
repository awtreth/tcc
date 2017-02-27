#ifndef MYJOINT_H
#define MYJOINT_H

#include <string>
#include <vector>
#include <map>
#include <memory>

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

    std::string toString() {
        std::string str;

        str += "pos = " + std::to_string(position) + "; ";
        str += "vel = " + std::to_string(velocity) + "; ";
        str += "torque = " + std::to_string(torque) + ";";

        return str;
    }

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

/**
 * @brief
 *
 */
typedef std::vector<Joint> JointVec;

#endif
