#include "MotionController.h"
#include <thread>
#include <chrono>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <string.h>

using namespace std::chrono;

template<class JointController, class JointCommand>
unsigned int MotionController<JointController,JointCommand>::getWritePeriod() const
{
    return writePeriod.count();
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::setWritePeriod(unsigned value)
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    writePeriod = std::chrono::microseconds(value);
    readPeriod = std::chrono::microseconds(value*readWritePeriodRatio);
    updateShiftPeriod();

    requestCount --;

    pauseMtx.unlock();
}

template<class JointController, class JointCommand>
unsigned int MotionController<JointController,JointCommand>::getReadPeriod() const
{
    return readPeriod.count();
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::setReadPeriod(unsigned value)
{

    requestCount++;
    std::unique_lock<std::mutex> lck(pauseMtx);

    readPeriod = std::chrono::microseconds(value);
    writePeriod = std::chrono::microseconds(value/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;
    pauseMtx.unlock();
}

template<class JointController, class JointCommand>
int MotionController<JointController,JointCommand>::getReadWritePeriodRatio() const
{
    return readWritePeriodRatio;
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::setReadWritePeriodRatio(int value)
{

    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    readWritePeriodRatio = value;//pow(2,value);
    writePeriod = std::chrono::microseconds(readPeriod/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;

    pauseMtx.unlock();

}

template<class JointController, class JointCommand>
double MotionController<JointController,JointCommand>::getReadWriteShift() const
{
    return readWriteShift;
}

template<class JointController, class JointCommand>
steady_clock::time_point MotionController<JointController,JointCommand>::getNextReadTime() const
{
    return nextReadTime;
}

template<class JointController, class JointCommand>
steady_clock::time_point MotionController<JointController,JointCommand>::getNextWriteTime() const
{
    return nextWriteTime;
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::updateShiftPeriod()
{
    this->shiftDuration = std::chrono::microseconds( (unsigned) (std::min(this->readPeriod.count(),this->writePeriod.count())*this->readWriteShift));
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::startIntervention()
{
    pauseMtx.lock();
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::stopIntervention()
{
    pauseMtx.unlock();
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::writeCmd(std::vector<JointCommand> cmd)
{

    writeJointController->sendCommand(cmd);
}

template<class JointController, class JointCommand>
std::vector<ReadJointCommand> MotionController<JointController, JointCommand>::onReadCmd()
{
    std::vector<ReadJointCommand> ret;

    //TODO: posso deixar fixo
    for(auto jmap : readJointController->getLastJointState())
        ret.push_back(ReadJointCommand(jmap.second.getName()));

    return ret;
}

template<class JointController, class JointCommand>
JointMap MotionController<JointController,JointCommand>::readCmd(std::vector<ReadJointCommand> cmd)
{
    readJointController->sendRequest(cmd);
    return readJointController->getLastJointState();
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::afterRead(JointMap updatedJoints)
{

}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::setReadWriteShift(double value)
{
    if(value > 0 && value < 1){

        requestCount++;

        std::unique_lock<std::mutex> lck(pauseMtx);

        readWriteShift = value;
        updateShiftPeriod();

        requestCount--;

        pauseMtx.unlock();

    }
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::resumeLoop(long readWaitTime)
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    this->nextReadTime = std::chrono::steady_clock::now() + std::chrono::microseconds(readWaitTime);
    this->nextWriteTime = this->nextReadTime + this->shiftDuration;

    //    clock_gettime(CLOCK_MONOTONIC,&nextReadTimeTs);

    //    nextReadTimeTs.tv_sec = nextReadTimeTs.tv_sec + (readWaitTime/(long)1E6 + (nextReadTimeTs.tv_nsec + (readWaitTime%(long)1E6)*(long)1E3)/(long)1E9);
    //    nextReadTimeTs.tv_nsec = (nextReadTimeTs.tv_nsec + (readWaitTime%(long)1e6)*(long)1e3)%(long)1e9;

    //    nextWriteTimeTs.tv_sec = nextReadTimeTs.tv_sec + (shiftDuration.count()/(long)1e6 + (nextReadTimeTs.tv_nsec + (shiftDuration.count()%(long)1e6)*(long)1e3)/(long)1e9);
    //    nextWriteTimeTs.tv_nsec = (nextReadTimeTs.tv_nsec + (shiftDuration.count()%(long)1e6)*(long)1e3)%(long)1e9;

    this->isPaused = false;

    pauseCv.notify_all();

    requestCount--;

    pauseMtx.unlock();

}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::pauseLoop()
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);


    this->isPaused = true;

    requestCount--;

    pauseMtx.unlock();

}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::close()
{

    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    isClosed = true;
    isPaused = false;

    pauseCv.notify_all();

    requestCount--;

    pauseMtx.unlock();

    if(mainThread.joinable())
        this->mainThread.join();

    //pthread_join(mainPThread,NULL);
}

template<class JointController, class JointCommand>
void* MotionController<JointController,JointCommand>::loop(void* param)
{
    MotionController *sync = (MotionController *) param;


    while(true) {

        std::unique_lock<std::mutex> lck(sync->pauseMtx);

        while(sync->isPaused || sync->requestCount>0 || sync->currentPageSet.hasFinished())
            sync->pauseCv.wait(lck);

        if (sync->isClosed)
            break;


        if(sync->nextReadTime < sync->nextWriteTime) {

            //sync->onRead();
            auto readCommands = sync->onReadCmd();

            std::this_thread::sleep_until(sync->nextReadTime);

            //            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sync->nextReadTimeTs, NULL);

            auto updatedJoints = sync->readCmd(readCommands);

            sync->afterRead(updatedJoints);

            sync->nextReadTime += sync->readPeriod;

            //            sync->nextReadTimeTs.tv_sec = sync->nextReadTimeTs.tv_sec + (sync->readPeriod.count()/(long)1e6 + (sync->nextReadTimeTs.tv_nsec + (sync->readPeriod.count()%(long)1e6)*(long)1e3)/(long)1e9);
            //            sync->nextReadTimeTs.tv_nsec = (sync->nextReadTimeTs.tv_nsec + (sync->readPeriod.count()%(long)1e6)*(long)1e3)%(long)1e9;


        }else {


            if(sync->currentPageSet.hasCurrentPose()){

                //std::vector<WriteJointCommand> wCmd;

                auto wCmd = sync->onWriteCmd(sync->currentPageSet.getCurrentPose());

                std::this_thread::sleep_until(sync->nextWriteTime);

                sync->writeCmd(wCmd);
                //            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sync->nextWriteTimeTs, NULL);

            }

            sync->currentPageSet.advanceTime(sync->getWritePeriod());

            sync->nextWriteTime += sync->writePeriod;

            //            sync->nextWriteTimeTs.tv_sec = sync->nextWriteTimeTs.tv_sec + (sync->writePeriod.count()/(long)1e6 + (sync->nextWriteTimeTs.tv_nsec + (sync->writePeriod.count()%(long)1e6)*(long)1e3)/(long)1e9);
            //            sync->nextWriteTimeTs.tv_nsec = (sync->nextWriteTimeTs.tv_nsec + (sync->writePeriod.count()%(long)1e6)*(long)1e3)%(long)1e9;
        }

        sync->pauseMtx.unlock();

    }

    return NULL;
}

template<class JointController, class JointCommand>
void MotionController<JointController,JointCommand>::initThread()
{
    setReadPeriod(50e3);

    this->isPaused = true;

    isClosed = false;

    mainThread = std::thread(&this->loop, this);

    mainThread.detach();

    struct sched_param param;

    param.__sched_priority = 51;

    pthread_setschedparam(mainThread.native_handle(), SCHED_FIFO, &param);

}

template<class JointController, class JointCommand>
MotionController<JointController,JointCommand>::MotionController(JointController *writeJointController_, IReadJointController *readJointController_){

    loadControllers(writeJointController_,readJointController_);
}

template<class JointController, class JointCommand>
bool MotionController<JointController,JointCommand>::loadControllers(JointController *writeJointController_, IReadJointController *readJointController_)
{
    writeJointController = std::shared_ptr<JointController>(writeJointController_);
    readJointController = std::shared_ptr<IReadJointController>(readJointController_);

    pageSetQueue = std::queue<PageSet>();

    //jointNames = _jointNames;

    initThread();

    return true;
}

template<class JointController, class JointCommand>
PageSet MotionController<JointController,JointCommand>::getCurrentPageSet() const
{
    return currentPageSet;
}

template<class JointController, class JointCommand>
std::queue<PageSet> MotionController<JointController,JointCommand>::getPageSetQueue() const
{
    return this->pageSetQueue;
}

template<class JointController, class JointCommand>
bool MotionController<JointController,JointCommand>::loadPageSet(PageSet pset)
{
    bool success = false;

    startIntervention();

    if(currentPageSet.hasFinished()){
        currentPageSet = pset;
        currentPageSet.resetTime();
        success = true;
    }

    stopIntervention();

    return success;
}

template<class JointController, class JointCommand>
bool MotionController<JointController,JointCommand>::startMotion()
{
    bool success = false;

    startIntervention();

    //FIXME: tem que verificar se o PageSet está resetado
    if(!currentPageSet.hasFinished()){
        currentPageSet.resetTime();
        stopIntervention();
        resumeLoop();
        success = true;
    }else
        stopIntervention();

    return success;
}

template<class JointController, class JointCommand>
MotionController<JointController,JointCommand>::MotionController()
{
}
