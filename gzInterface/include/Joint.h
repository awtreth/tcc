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
class JointState {

    public:

    double position = 0; /**< TODO: describe */

    double velocity = 0; /**< TODO: describe */

    double torque = 0; /**< TODO: describe */

    /**
     * @brief
     *
     */
    JointState() {}

    /**
     * @brief
     *
     * @param _position
     * @param _velocity
     * @param _torque
     */
    JointState(double _position, double _velocity, double _torque) {
        position = _position;
        velocity = _velocity;
        torque = _torque;
    }

};

//class Joint {

//    private:

//    //int id = 0;

//    std::string name = "";

//    JointState jointState;

//    public:

//    Joint(){}


//    Joint(std::string _name) {
//        //id = _id;
//        name = _name;
//    }

//    //int getId() { return id; }

//    std::string getName() { return name; }

//    JointState getJointState() { return jointState; }

//    void setJointState(JointState newJointState) { jointState = newJointState; }

//};

/**
 * @brief
 *
 */
typedef std::map<std::string,JointState> JointStateMap ;

/**
 * @brief
 *
 */
typedef std::vector<JointState> JointStateVec;

#endif
