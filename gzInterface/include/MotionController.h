#ifndef MotionController_H
#define MotionController_H

#include <JointController.h>
#include <Joint.h>
#include <JointCommand.h>
#include <chrono>
#include <thread>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <pthread.h>
#include <time.h>
#include <queue>
#include <PageSet.h>
#include <Page.h>
#include <Pose.h>

using namespace std::chrono;

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

    void initThread();

    protected:

    steady_clock::time_point getNextReadTime() const;

    steady_clock::time_point getNextWriteTime() const;

//    virtual void onRead(){}

//    virtual void read()=0;

//    virtual void afterRead(){}

//    virtual void onWrite(){}

//    virtual void write()=0;

//    virtual void afterWrite(){}

//    virtual void onReadMiss(){}

//    virtual void onWriteMiss(){}

    void startIntervention();

    void stopIntervention();

    JointControllerPtr jointController;
    //std::vector<Joint> joints;

    PageSet currentPageSet;
    std::mutex currentPageSetMtx;

    std::queue<PageSet> pageSetQueue;
    std::mutex pageSetQueueMtx;

    virtual std::vector<WriteJointCommandPtr> onWriteCmd(Pose currentPose);
    virtual void writeCmd(std::vector<WriteJointCommandPtr> cmd);
    virtual std::vector<ReadJointCommandPtr> onReadCmd();
    virtual std::vector<Joint> readCmd(std::vector<ReadJointCommandPtr> cmd);
    virtual void afterRead(std::vector<Joint> updatedJoints);

    void resumeLoop(long readWaitTime = 0);

    void pauseLoop();

    void close();

    public:

    MotionController();

    MotionController(JointControllerPtr _jointController);//10ms

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




#endif
