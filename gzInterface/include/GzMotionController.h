#ifndef GZMOTIONCONTROLLER_H
#define GZMOTIONCONTROLLER_H

#include <AbsMotionController.h>
#include <GzJointController.h>
#include <iostream>
#include <memory>

class GzMotionController : public AbsMotionController {


    // AbsMotionController interface
    private:

    virtual std::vector<JointCommandPtr> onWrite()
    {
        std::cout << "onWrite" << std::endl;

        return std::vector<JointCommandPtr>();

    }
    virtual void afterRead()
    {
        std::cout << "afterRead" << std::endl;
    }

    virtual void onReadMiss()
    {
    }

    virtual void onWriteMiss()
    {
    }

    public:

    GzMotionController(GzJointController _jointController) : AbsMotionController (std::make_shared<GzJointController>(_jointController)) {}


};




#endif
