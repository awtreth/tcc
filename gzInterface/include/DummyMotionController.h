#ifndef DUMMY_MOTION_CONTROLLER_H
#define DUMMY_MOTION_CONTROLLER_H

#include <AbsMotionController.h>
#include <JointController.h>
#include <iostream>
#include <memory>

class DummyMotionController : public AbsMotionController {

    public:

    DummyMotionController():AbsMotionController(){}


    // AbsMotionController interface
    protected:

    virtual void onRead()
    {
        std::cout << "onRead" << std::endl;
    }

    virtual void read()
    {
        std::cout << "read" << std::endl;
    }

    virtual void afterRead()
    {
        std::cout << "afterRead" << std::endl;
    }

    virtual void onWrite()
    {
        std::cout << "onWrite" << std::endl;
    }

    virtual void write()
    {
        std::cout << "write" << std::endl;
    }

    virtual void afterWrite()
    {
        std::cout << "afterWrite" << std::endl;
    }

    virtual void onReadMiss()
    {
        std::cout << "onReadMiss" << std::endl;
    }

    virtual void onWriteMiss()
    {
        std::cout << "onWriteMiss" << std::endl;
    }
};




#endif
