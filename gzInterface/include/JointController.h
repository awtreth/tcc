#ifndef IJOINTPOSVELCONTROLLER_H
#define IJOINTPOSVELCONTROLLER_H

#include <vector>
#include <string>
#include <Joint.h>


/**
 * @brief
 *
 */
class IJointPosVelController {

    public:

    /**
     * @brief
     *
     * @param pos
     * @param vel
     * @param jointID
     * @return bool
     */
    virtual bool goPosVel(double pos, double vel, int jointID = 0) = 0;

    /**
     * @brief
     *
     * @param pos
     * @param vel
     * @param jointName
     * @return bool
     */
    virtual bool goPosVel(double pos, double vel, std::string jointName = "") = 0;

    /**
     * @brief
     *
     * @param std::vector<JointPosVelCommand>
     * @return bool
     */
    virtual bool goPosVel(std::vector<JointPosVelCommand>) = 0;
};

/**
 * @brief
 *
 */
class IJointTorqueController {

    public:

    /**
     * @brief
     *
     * @param pos
     * @param vel
     * @param jointID
     * @return bool
     */
    virtual bool goTorque(double torque, int jointID = 0) = 0;

    /**
     * @brief
     *
     * @param pos
     * @param vel
     * @param jointName
     * @return bool
     */
    virtual bool goTorque(double torque, std::string jointName = "") = 0;

    /**
     * @brief
     *
     * @param std::vector<JointTorqueCommand>
     * @return bool
     */
    virtual bool goTorque(std::vector<JointTorqueCommand>) = 0;
};

/**
 * @brief
 *
 */
class AbsJointController {

    protected:

    JointStateVec jointStateVec; /**< TODO: describe */

    JointStateMap jointStateMap; /**< TODO: describe */

    public:

    /**
     * @brief
     *
     * @param jointNames
     */
    AbsJointController(std::vector<std::string> jointNames){

        for(auto jointName: jointNames) {
            jointStateMap[jointName] = JointState();
            jointStateVec.push_back(JointState());
        }
    }

    /**
     * @brief
     *
     * @return int
     */
    int nJoints() { return jointStateVec.size(); }

    /**
     * @brief
     *
     * @param jointID
     * @return JointState
     */
    JointState getLastJointState(int jointID = 0){ return jointStateVec[jointID]; }

    /**
     * @brief
     *
     * @param jointName
     * @return JointState
     */
    JointState getLastJointState(std::string jointName = ""){return jointStateMap[jointName];}

    /**
     * @brief
     *
     * @return JointStateVec
     */
    JointStateVec getLastJointStateVec(){ return jointStateVec; }

    /**
     * @brief
     *
     * @return JointStateMap
     */
    JointStateMap getLastJointStateMap() { return jointStateMap; }

    /**
     * @brief
     *
     * @return bool
     */
    virtual bool readJointStates() = 0;

};


#endif
