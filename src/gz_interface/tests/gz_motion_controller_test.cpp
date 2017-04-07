#include <GzJointController.h>
#include <vector>
#include <string>
#include <PosVelWriteJointCommand.h>
#include <TorqueWriteJointCommand.h>
#include <GzMotionController.h>
#include <iostream>
#include <cmath>

#define JOINT_1 "arm_sholder_pan_joint"
#define JOINT_2 "arm_elbow_pan_joint"

int main(){

    std::vector<std::string> jointVec;

    jointVec.push_back(JOINT_1);
    jointVec.push_back(JOINT_2);

    GzJointController cont(jointVec,"~/simple_arm/writeRequest", "~/simple_arm/readRequest","~/simple_arm/readResponse");

    Page page;

    page.setModelName("base");
    page.setMotionName("defaultSinusoid");

    const int nPoints = 50;
    const int period = 2;//segundos

    for(int i = 0; i < nPoints; i++){
        Pose pose;
        pose.addPosVel(PosVel(cos(((double)i)/nPoints*2*M_PI),-sin(((double)i)/nPoints*2*M_PI),JOINT_1));
        pose.addPosVel(PosVel(sin(((double)i)/nPoints*2*M_PI), cos(((double)i)/nPoints*2*M_PI),JOINT_2));
        pose.setTimeToNext(1./nPoints*period*1e6);
        page.addPose(pose);
    }


    page.setTimesByTimeToNext();
    page.setNumberOfLoops(0);
    std::cout << page.computePageDuration() << std::endl;

    PageSet pset;

    pset.setPage(page);

    pset.toString();

    GzMotionController mc(&cont);

    mc.loadPageSet(pset);

    mc.startMotion();

    sleep(1000);

    return 0;
}
