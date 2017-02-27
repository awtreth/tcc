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

unsigned int MotionController::getWritePeriod() const
{
    return writePeriod.count();
}

void MotionController::setWritePeriod(unsigned value)
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    writePeriod = std::chrono::microseconds(value);
    readPeriod = std::chrono::microseconds(value*readWritePeriodRatio);
    updateShiftPeriod();

    requestCount --;

    pauseMtx.unlock();
}

unsigned int MotionController::getReadPeriod() const
{
    return readPeriod.count();
}

void MotionController::setReadPeriod(unsigned value)
{

    requestCount++;
    std::unique_lock<std::mutex> lck(pauseMtx);

    readPeriod = std::chrono::microseconds(value);
    writePeriod = std::chrono::microseconds(value/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;
    pauseMtx.unlock();
}

int MotionController::getReadWritePeriodRatio() const
{
    return readWritePeriodRatio;
}

void MotionController::setReadWritePeriodRatio(int value)
{

    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    readWritePeriodRatio = value;//pow(2,value);
    writePeriod = std::chrono::microseconds(readPeriod/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;

    pauseMtx.unlock();

}

double MotionController::getReadWriteShift() const
{
    return readWriteShift;
}

steady_clock::time_point MotionController::getNextReadTime() const
{
    return nextReadTime;
}

steady_clock::time_point MotionController::getNextWriteTime() const
{
    return nextWriteTime;
}

void MotionController::updateShiftPeriod()
{
    this->shiftDuration = std::chrono::microseconds( (unsigned) (std::min(this->readPeriod.count(),this->writePeriod.count())*this->readWriteShift));
}

void MotionController::startIntervention()
{
    pauseMtx.lock();
}

void MotionController::stopIntervention()
{
    pauseMtx.unlock();
}

std::vector<WriteJointCommandPtr> MotionController::onWriteCmd(Pose currentPose)
{
    auto posVelCmds = currentPose.toJointCommand();
    auto ret = std::vector<WriteJointCommandPtr>();

    for(auto posVelCmd : posVelCmds)
        ret.push_back(std::make_shared<PosVelWriteJointCommand>(posVelCmd));

    return ret;
}

void MotionController::writeCmd(std::vector<WriteJointCommandPtr> cmd)
{
    jointController->sendWriteCommand(cmd);
}

std::vector<ReadJointCommandPtr> MotionController::onReadCmd()
{
    auto ret = std::vector<ReadJointCommandPtr>();

    for(Joint joint : jointController->getJointVec())
        ret.push_back(std::make_shared<ReadJointCommand>(ReadJointCommand(joint.getName())));

    return ret;
}

std::vector<Joint> MotionController::readCmd(std::vector<ReadJointCommandPtr> cmd)
{
    return jointController->sendReadCommand(cmd);
}

void MotionController::afterRead(std::vector<Joint> updatedJoints)
{

}

void MotionController::setReadWriteShift(double value)
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

void MotionController::resumeLoop(long readWaitTime)
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

void MotionController::pauseLoop()
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);


    this->isPaused = true;

    requestCount--;

    pauseMtx.unlock();

}

void MotionController::close()
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

void* MotionController::loop(void* param)
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

                std::vector<WriteJointCommandPtr> wCmd;

                wCmd = sync->onWriteCmd(sync->currentPageSet.getCurrentPose());

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

void MotionController::initThread()
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


MotionController::MotionController(JointControllerPtr _jointController){
    jointController = _jointController;
    //joints = jointController->getJointVec();
    pageSetQueue = std::queue<PageSet>();

    initThread();
}

PageSet MotionController::getCurrentPageSet() const
{
    return currentPageSet;
}

std::queue<PageSet> MotionController::getPageSetQueue() const
{
    return this->pageSetQueue;
}

bool MotionController::loadPageSet(PageSet pset)
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

bool MotionController::startMotion()
{
    bool success = false;

    startIntervention();

    //FIXME: tem que verificar se o PageSet está resetado
    if(!currentPageSet.hasFinished()){
        stopIntervention();
        resumeLoop();
        success = true;
    }else
        stopIntervention();

    return success;
}

MotionController::MotionController()
{

    jointController = JointControllerPtr();
    currentPageSet = PageSet();
    pageSetQueue = std::queue<PageSet>();

    initThread();
}
