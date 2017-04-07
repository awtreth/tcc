#ifndef MotionController_H
#define MotionController_H

#include <IWriteJointController.h>
#include <IReadJointController.h>
#include <Joint.h>
#include <JointCommand.h>
#include <chrono>
#include <thread>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <pthread.h>
#include <time.h>
#include <queue>
#include <joint_motion/PageSet.h>
#include <joint_motion/Page.h>
#include <joint_motion/Pose.h>

using namespace std::chrono;

template<typename JointController, typename JointCommand>
class MotionController {

    private:


    std::thread mainThread;

    pthread_t mainPThread;

    std::chrono::microseconds writePeriod;//por padrão: 50ms

    std::chrono::microseconds readPeriod;

    int readWritePeriodRatio = 1;

    double readWriteShift = 0.5;

    std::chrono::microseconds shiftDuration;

    steady_clock::time_point nextReadTime;

    steady_clock::time_point nextWriteTime;


    timespec nextReadTimeTs;

    timespec nextWriteTimeTs;


    bool isPaused = true;

    int requestCount = 0;

    std::mutex pauseMtx;

    std::mutex nextTimeMtx;

    std::condition_variable pauseCv;

    void updateShiftPeriod();

    bool isClosed = false;

    static void* loop(void*);


    protected:
    void initThread();

    steady_clock::time_point getNextReadTime() const;

    steady_clock::time_point getNextWriteTime() const;

    void startIntervention();

    void stopIntervention();

    //WriteJointControllerPtr writeJointController;
    std::shared_ptr<JointController> writeJointController;
    ReadJointControllerPtr readJointController;

    PageSet currentPageSet;
    std::mutex currentPageSetMtx;

    std::queue<PageSet> pageSetQueue;
    std::mutex pageSetQueueMtx;

    virtual std::vector<JointCommand> onWriteCmd(Pose currentPose){return std::vector<JointCommand>();}
    virtual void writeCmd(std::vector<JointCommand> cmd);
    virtual std::vector<ReadJointCommand> onReadCmd();
    virtual JointMap readCmd(std::vector<ReadJointCommand> cmd);
    virtual void afterRead(JointMap updatedJoints);

    void resumeLoop(long readWaitTime = 0);

    void pauseLoop();

    std::vector<std::string> jointNames;

    public:
    void close();

    MotionController();

    MotionController(JointController* writeJointController_, IReadJointController* readJointController_);//10ms

    bool loadControllers(JointController* writeJointController_, IReadJointController* readJointController_);

    //Funcionalidades de controle de movimento
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

    //Funcionalidades Básica de controle de loop
    unsigned int getWritePeriod() const;
    void setWritePeriod(unsigned int value);
    unsigned int getReadPeriod() const;
    void setReadPeriod(unsigned int value);
    int getReadWritePeriodRatio() const;
    void setReadWritePeriodRatio(int value);
    double getReadWriteShift() const;
    void setReadWriteShift(double value);

};

#include <IPosVelJointController.h>

template class MotionController<IPosVelJointController, PosVelWriteJointCommand>;

class PosVelMotionController : public MotionController<IPosVelJointController, PosVelWriteJointCommand> {

    // MotionController interface
protected:

    PosVelMotionController():MotionController<IPosVelJointController, PosVelWriteJointCommand>(){}

    PosVelMotionController(IPosVelJointController* wjc, IReadJointController* rjc):MotionController<IPosVelJointController, PosVelWriteJointCommand>(wjc,rjc){}

    virtual std::vector<PosVelWriteJointCommand> onWriteCmd(Pose currentPose) override{
        return currentPose.toJointCommand();
    }
};


#endif
