#ifndef ABSJOINTCONTROLLERMANAGER_H
#define ABSJOINTCONTROLLERMANAGER_H

#include <JointController.h>


class AbsJointControllerManager {

    private:

    ITorqueJointControllerPtr torqueJointController;

    IPosVelJointControllerPtr posVelJointController;

    double writePeriod = 1;

    double readPeriod = 1;

    int readWritePeriodRatio = 1;

    double readWriteShift = 0.5;

    virtual void onWrite() = 0;

    public:

    AbsJointControllerManager(ITorqueJointControllerPtr &_torqueJointController);

};




#endif
