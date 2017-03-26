#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include <string>
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


#endif
