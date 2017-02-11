#include "AbsReadWriteSynchronizer.h"
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

unsigned int AbsReadWriteSynchronizer::getWritePeriod() const
{
    return writePeriod.count();
}

void AbsReadWriteSynchronizer::setWritePeriod(unsigned value)
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    writePeriod = std::chrono::microseconds(value);
    readPeriod = std::chrono::microseconds(value*readWritePeriodRatio);
    updateShiftPeriod();

    requestCount --;

    pauseMtx.unlock();
}

unsigned int AbsReadWriteSynchronizer::getReadPeriod() const
{
    return readPeriod.count();
}

void AbsReadWriteSynchronizer::setReadPeriod(unsigned value)
{

    requestCount++;
    std::unique_lock<std::mutex> lck(pauseMtx);

    readPeriod = std::chrono::microseconds(value);
    writePeriod = std::chrono::microseconds(value/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;
    pauseMtx.unlock();
}

int AbsReadWriteSynchronizer::getReadWritePeriodRatio() const
{
    return readWritePeriodRatio;
}

void AbsReadWriteSynchronizer::setReadWritePeriodRatio(int value)
{

    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    readWritePeriodRatio = pow(2,value);
    writePeriod = std::chrono::microseconds(readPeriod/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;

    pauseMtx.unlock();

}

double AbsReadWriteSynchronizer::getReadWriteShift() const
{
    return readWriteShift;
}

steady_clock::time_point AbsReadWriteSynchronizer::getNextReadTime() const
{
    return nextReadTime;
}

steady_clock::time_point AbsReadWriteSynchronizer::getNextWriteTime() const
{
    return nextWriteTime;
}

void AbsReadWriteSynchronizer::updateShiftPeriod()
{
    this->shiftDuration = std::chrono::microseconds( (unsigned) (std::min(this->readPeriod.count(),this->writePeriod.count())*this->readWriteShift));
}

void AbsReadWriteSynchronizer::startIntervention()
{
    pauseMtx.lock();
}

void AbsReadWriteSynchronizer::stopIntervention()
{
    pauseMtx.unlock();
}

void AbsReadWriteSynchronizer::setReadWriteShift(double value)
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

void AbsReadWriteSynchronizer::resumeLoop(long readWaitTime)
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

void AbsReadWriteSynchronizer::pauseLoop()
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);


    this->isPaused = true;

    requestCount--;

    pauseMtx.unlock();

}

void AbsReadWriteSynchronizer::close()
{

    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    isClosed = true;
    isPaused = false;

    pauseCv.notify_all();

    requestCount--;

    pauseMtx.unlock();

    this->mainThread.join();

    //pthread_join(mainPThread,NULL);
}

void AbsReadWriteSynchronizer::loop(void* param)
{
    AbsReadWriteSynchronizer *sync = (AbsReadWriteSynchronizer *) param;


    while(true) {

        std::unique_lock<std::mutex> lck(sync->pauseMtx);

        while(sync->isPaused || sync->requestCount>0)
            sync->pauseCv.wait(lck);

        if (sync->isClosed)
            break;


        if(sync->nextReadTime < sync->nextWriteTime) {

            sync->onRead();

            std::this_thread::sleep_until(sync->nextReadTime);

            //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sync->nextReadTimeTs, NULL);

            sync->read();

            sync->afterRead();

            sync->nextReadTime += sync->readPeriod;

//            sync->nextReadTimeTs.tv_sec = sync->nextReadTimeTs.tv_sec + (sync->readPeriod.count()/(long)1e6 + (sync->nextReadTimeTs.tv_nsec + (sync->readPeriod.count()%(long)1e6)*(long)1e3)/(long)1e9);
//            sync->nextReadTimeTs.tv_nsec = (sync->nextReadTimeTs.tv_nsec + (sync->readPeriod.count()%(long)1e6)*(long)1e3)%(long)1e9;


        }else {

            sync->onWrite();

            std::this_thread::sleep_until(sync->nextWriteTime);

//            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sync->nextWriteTimeTs, NULL);

            sync->write();

            sync->afterWrite();

            sync->nextWriteTime += sync->writePeriod;


//            sync->nextWriteTimeTs.tv_sec = sync->nextWriteTimeTs.tv_sec + (sync->writePeriod.count()/(long)1e6 + (sync->nextWriteTimeTs.tv_nsec + (sync->writePeriod.count()%(long)1e6)*(long)1e3)/(long)1e9);
//            sync->nextWriteTimeTs.tv_nsec = (sync->nextWriteTimeTs.tv_nsec + (sync->writePeriod.count()%(long)1e6)*(long)1e3)%(long)1e9;
        }

        sync->pauseMtx.unlock();

    }

//    return NULL;
}


AbsReadWriteSynchronizer::AbsReadWriteSynchronizer()
{

    this->isPaused = true;

    isClosed = false;

    mainThread = std::thread(&this->loop, this);

    struct sched_param param;

    param.__sched_priority = 99;

    std::cout << pthread_setschedparam(mainThread.native_handle(), SCHED_RR, &param) << std::endl;

//    int error;
//    struct sched_param param;
//    pthread_attr_t attr;

//    pthread_attr_init(&attr);

//    error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
//    if (error != 0)
//        printf("pthread_attr_setschedpolicy error = %d\n", error);
//    error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
//    if (error != 0)
//        printf("pthread_attr_setinheritsched error = %d\n", error);

//    memset(&param, 0, sizeof(param));
//    param.sched_priority = 31;    // RT
//    error = pthread_attr_setschedparam(&attr, &param);
//    if (error != 0)
//        printf("pthread_attr_setschedparam error = %d\n", error);

//    // create and start the thread
//    if ((error = pthread_create(&this->mainPThread, &attr, this->loop, this)) != 0)
//    {
//        printf("Creating timer thread failed!!");
//        exit(-1);
//    }

}
