#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <JointController.h>
#include <AbsReadWriteSynchronizer.h>
#include <PageSet.h>
#include <JointCommand.h>
#include <Joint.h>
#include <queue>
#include <mutex>

class MotionController : public AbsReadWriteSynchronizer{

    private:

    std::vector<WriteJointCommandPtr> wCmd;
    std::vector<ReadJointCommandPtr> rCmd;
    std::vector<Joint> joints;

    // AbsReadWriteSynchronizer interface
    void onWrite();
    void write();
    //void afterWrite();

    void onRead();
    void read();
    void afterRead();

    protected:

    JointControllerPtr jointController;

    PageSet currentPageSet;
    std::mutex currentPageSetMtx;

    std::queue<PageSet> pageSetQueue;
    std::mutex pageSetQueueMtx;


    virtual std::vector<WriteJointCommandPtr> onWriteCmd(Pose currentPose);
    virtual void writeCmd(std::vector<WriteJointCommandPtr> cmd);
    virtual std::vector<ReadJointCommandPtr> onReadCmd();
    virtual std::vector<Joint> readCmd(std::vector<ReadJointCommandPtr> cmd);
    virtual void afterRead(std::vector<Joint> updatedJoints);

    public:


    MotionController();

    MotionController(JointControllerPtr _jointController);//10ms

    PageSet getCurrentPageSet() const;

    std::queue<PageSet> getPageSetQueue() const;

    bool loadPageSet(PageSet pset);

    bool clearPageSetQueue();

    bool appendPageSet(PageSet pset);

    bool startNextMotion(bool stopMotion = false, bool waitPageEnd = true);

    bool changePageSet(PageSet pset, bool stopMotion = false, bool waitPageEnd = true);

    bool startMotion();

    bool isMoving();

    bool stopMotion(bool waitPageEnd = true);


};




#endif
