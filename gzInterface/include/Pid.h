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



#endif
