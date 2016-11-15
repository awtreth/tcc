#ifndef IJOINTPOSVELCONTROLLER_H
#define IJOINTPOSVELCONTROLLER_H

#include <vector>
#include <string>
#include <Joint.h>


/**
 * @brief
 *
 */
class IPosVelJointController {

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
class ITorqueJointController {

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

    JointVec jointVec;

    std::map<std::string,int> jointNamesMap;

    public:

    /**
     * @brief
     *
     * @param jointNames
     */
    AbsJointController(std::vector<std::string> jointNames){

        int i = 0;

        for(auto jointName: jointNames) {

            jointNamesMap[jointName] = i++;
            jointVec.push_back(Joint(jointName));

        }
    }

    /**
     * @brief
     *
     * @return int
     */
    int nJoints() { return jointVec.size(); }

    /**
     * @brief
     *
     * @param jointID
     * @return JointState
     */
    JointState getLastJointState(int jointID = 0){ return jointVec[jointID].getJointState(); }

    /**
     * @brief
     *
     * @param jointName
     * @return JointState
     */
    JointState getLastJointState(std::string jointName = ""){return jointVec[jointNamesMap[jointName]].getJointState();} //TODO: tratar jointName=""

    /**
     * @brief
     *
     * @return JointStateVec
     */
    std::vector<JointState> getLastJointStates() {

        std::vector<JointState> output;

        for(Joint joint : jointVec) {
            output.push_back(joint.getJointState());
        }

        return output;
    }


    /**
     * @brief
     *
     * @return bool
     */
    virtual bool readJointStates() = 0;

};

class PidAbsJointController : public AbsJointController {

    public:

    virtual bool setPosPid(PidValues pid, int jointID = 0) { jointVec[jointID].setPosPid(pid); return true;}

    virtual bool setPosPid(PidValues pid, std::string jointName = "") { jointVec[jointNamesMap[jointName]].setPosPid(pid); return true; }

    virtual bool setPosPid(std::vector<PidValues> pids) {
        for(unsigned int i = 0; i < pids.size(); i++) {
            jointVec[i].setPosPid(pids[i]);
        }
        return true;
    }

    virtual bool setVelPid(PidValues pid, int jointID = 0) {jointVec[jointID].setVelPid(pid); return true;}

    virtual bool setVelPid(PidValues pid, std::string jointName = "") { jointVec[jointNamesMap[jointName]].setVelPid(pid); return true; }

    virtual bool setVelPid(std::vector<PidValues> pids) {
        for(unsigned int i = 0; i < pids.size(); i++) {
            jointVec[i].setVelPid(pids[i]);
        }
        return true;
    }


    virtual PidValues getPosPid(int jointID = 0) { return jointVec[jointID].getPosPid(); }

    virtual PidValues getPosPid(std::string jointName = "") { return jointVec[jointNamesMap[jointName]].getPosPid(); }

    virtual std::vector<PidValues> getPosPids() {
        std::vector<PidValues> output;

        for(Joint joint : jointVec) {
            output.push_back(joint.getPosPid());
        }

        return output;
    }

    virtual PidValues getVelPid(int jointID = 0) {return jointVec[jointID].getVelPid();}

    virtual PidValues getVelPid(std::string jointName = "") { return jointVec[jointNamesMap[jointName]].getPosPid(); }

    virtual std::vector<PidValues> getVelPids() {
        std::vector<PidValues> output;

        for(Joint joint : jointVec) {
            output.push_back(joint.getVelPid());
        }

        return output;
    }



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

    // AbsJointController interface
    virtual bool readJointStates() = 0;
};


#endif
