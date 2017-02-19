#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include <vector>
#include <string>
#include <Joint.h>
#include <memory>
#include <JointCommand.h>
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
    virtual bool goPosVel(std::vector<PosVelWriteJointCommand>) = 0;

};

typedef std::shared_ptr<IPosVelJointController> PosVelJointControllerPtr;


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
    virtual bool goTorque(std::vector<TorqueWriteJointCommand>) = 0;
};

typedef std::shared_ptr<ITorqueJointController> ITorqueJointControllerPtr;


/**
 * @brief
 *
 */
class AbsJointController {

    protected:

    JointVec jointVec; /**< TODO: describe */

    std::map<std::string,int> jointNamesMap; /**< TODO: describe */

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

        jointNamesMap[""] = 0;
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

    //virtual

    virtual std::vector<Joint> sendReadCommand(std::vector<ReadJointCommandPtr> cmd) = 0;

    virtual bool sendWriteCommand(std::vector<WriteJointCommandPtr> cmd) = 0;

    JointVec getJointVec() const{
        return jointVec;
    }
};

typedef std::shared_ptr<AbsJointController> JointControllerPtr;



/**
 * @brief
 *
 */
class AbsPidJointController : public AbsJointController {

    public:

    AbsPidJointController(std::vector<std::string> jointNames) : AbsJointController (jointNames) {}


    /**
     * @brief
     *
     * @param pid
     * @param jointID
     * @return bool
     */
    virtual bool setPosPid(PidValues pid, int jointID = 0) { jointVec[jointID].setPosPid(pid); return true;}

    /**
     * @brief
     *
     * @param pid
     * @param jointName
     * @return bool
     */
    virtual bool setPosPid(PidValues pid, std::string jointName = "") { jointVec[jointNamesMap[jointName]].setPosPid(pid); return true; }

    /**
     * @brief
     *
     * @param pids
     * @return bool
     */
    virtual bool setPosPid(std::vector<PidValues> pids) {
        for(unsigned int i = 0; i < pids.size(); i++) {
            jointVec[i].setPosPid(pids[i]);
        }
        return true;
    }

    /**
     * @brief
     *
     * @param pid
     * @param jointID
     * @return bool
     */
    virtual bool setVelPid(PidValues pid, int jointID = 0) {jointVec[jointID].setVelPid(pid); return true;}

    /**
     * @brief
     *
     * @param pid
     * @param jointName
     * @return bool
     */
    virtual bool setVelPid(PidValues pid, std::string jointName = "") { jointVec[jointNamesMap[jointName]].setVelPid(pid); return true; }

    /**
     * @brief
     *
     * @param pids
     * @return bool
     */
    virtual bool setVelPid(std::vector<PidValues> pids) {
        for(unsigned int i = 0; i < pids.size(); i++) {
            jointVec[i].setVelPid(pids[i]);
        }
        return true;
    }

    virtual bool setPosVelPid(PidValues posPid, PidValues velPid, int jointID = 0) {
        return AbsPidJointController::setPosPid(posPid, jointID) && AbsPidJointController::setVelPid(velPid, jointID);
    }

    virtual bool setPosVelPid(PidValues posPid, PidValues velPid, std::string jointName = 0) {
        return AbsPidJointController::setPosPid(posPid, jointName) && AbsPidJointController::setVelPid(velPid, jointName);
    }

    virtual bool setPosVelPid(std::vector<PidValues> posPids, std::vector<PidValues> velPids) {
        return AbsPidJointController::setPosPid(posPids) && AbsPidJointController::setPosPid(velPids);
    }




    /**
     * @brief
     *
     * @param jointID
     * @return PidValues
     */
    virtual PidValues getPosPid(int jointID = 0) { return jointVec[jointID].getPosPid(); }

    /**
     * @brief
     *
     * @param jointName
     * @return PidValues
     */
    virtual PidValues getPosPid(std::string jointName = "") { return jointVec[jointNamesMap[jointName]].getPosPid(); }

    /**
     * @brief
     *
     * @return std::vector<PidValues>
     */
    virtual std::vector<PidValues> getPosPids() {
        std::vector<PidValues> output;

        for(Joint joint : jointVec) {
            output.push_back(joint.getPosPid());
        }

        return output;
    }

    /**
     * @brief
     *
     * @param jointID
     * @return PidValues
     */
    virtual PidValues getVelPid(int jointID = 0) {return jointVec[jointID].getVelPid();}

    /**
     * @brief
     *
     * @param jointName
     * @return PidValues
     */
    virtual PidValues getVelPid(std::string jointName = "") { return jointVec[jointNamesMap[jointName]].getPosPid(); }

    /**
     * @brief
     *
     * @return std::vector<PidValues>
     */
    virtual std::vector<PidValues> getVelPids() {
        std::vector<PidValues> output;

        for(Joint joint : jointVec) {
            output.push_back(joint.getVelPid());
        }

        return output;
    }


    // AbsJointController interface
    /**
     * @brief
     *
     * @return bool
     */
    virtual bool readJointStates() = 0;

    virtual std::vector<Joint> sendReadCommand(std::vector<ReadJointCommandPtr> cmd) = 0;

    // AbsJointController interface
    virtual bool sendWriteCommand(std::vector<WriteJointCommandPtr> cmd) = 0;
};


#endif
