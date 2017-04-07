#include <iostream>
#include <gz_interface/GzJointController.h>
#include <MotionController.h>

int main(int _argc, char **_argv)
{
    std::vector<std::string> jointVec;

    jointVec.push_back("arm_sholder_pan_joint");
    jointVec.push_back("arm_elbow_pan_joint");

    GzJointController controller(jointVec,"~/simple_arm/writeRequest", "~/simple_arm/readRequest","~/simple_arm/readResponse");

    MotionController motionController(&controller, &controller);


    return 0;
}
