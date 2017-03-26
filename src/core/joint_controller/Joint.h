#ifndef MYJOINT_H
#define MYJOINT_H

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <JointState.h>
#include <PidValues.h>

/**
 * @brief Contém informações básicas de uma junta. Resultado genérico de leitura
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

    double getPos() const{return jointState.position;}

    double setPos(const double pos){jointState.position = pos;}

    double getVel() const{return jointState.velocity;}

    double setVel(const double vel){jointState.velocity = vel;}

    double getTorque() const{return jointState.torque;}

    double setTorque(const double torque){jointState.torque = torque;}

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
    void setPosPid(double kp, double ki, double kd){posPID.kp=kp;posPID.ki=ki;posPID.kd=kd;}

    /**
     * @brief
     *
     * @param kp
     * @param ki
     * @param kd
     */
    void setVelPid(double kp, double ki, double kd){velPID.kp=kp;velPID.ki=ki;velPID.kd=kd;}


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

typedef std::map<std::string,Joint> JointMap;

#endif
