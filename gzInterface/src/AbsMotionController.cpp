#include "AbsMotionController.h"
#include <thread>
#include <chrono>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

using namespace std::chrono;

unsigned int AbsMotionController::getWritePeriod() const
{
    return writePeriod.count();
}

void AbsMotionController::setWritePeriod(unsigned value)
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    writePeriod = std::chrono::microseconds(value);
    readPeriod = std::chrono::microseconds(value*readWritePeriodRatio);
    updateShiftPeriod();

    requestCount --;

    pauseMtx.unlock();
}

unsigned int AbsMotionController::getReadPeriod() const
{
    return readPeriod.count();
}

void AbsMotionController::setReadPeriod(unsigned value)
{

    requestCount++;
    std::unique_lock<std::mutex> lck(pauseMtx);

    readPeriod = std::chrono::microseconds(value);
    writePeriod = std::chrono::microseconds(value/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;
    pauseMtx.unlock();
}

int AbsMotionController::getReadWritePeriodRatio() const
{
    return readWritePeriodRatio;
}

void AbsMotionController::setReadWritePeriodRatio(int value)
{

    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    readWritePeriodRatio = pow(2,value);
    writePeriod = std::chrono::microseconds(readPeriod/readWritePeriodRatio);
    updateShiftPeriod();

    requestCount--;

    pauseMtx.unlock();

}

double AbsMotionController::getReadWriteShift() const
{
    return readWriteShift;
}

void AbsMotionController::updateShiftPeriod()
{
    this->shiftDuration = std::chrono::microseconds( (unsigned) (std::min(this->readPeriod.count(),this->writePeriod.count())*this->readWriteShift));
}

void AbsMotionController::startIntervention()
{
    pauseMtx.lock();
}

void AbsMotionController::stopIntervention()
{
    pauseMtx.unlock();
}

void AbsMotionController::setReadWriteShift(double value)
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

void AbsMotionController::resume(unsigned int readWaitTime)
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    this->nextReadTime = std::chrono::steady_clock::now() + std::chrono::microseconds(readWaitTime);
    this->nextWriteTime = this->nextReadTime + this->shiftDuration;

    this->isPaused = false;

    pauseCv.notify_all();

    requestCount--;

    pauseMtx.unlock();

}

void AbsMotionController::pause()
{
    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);


    this->isPaused = true;

    requestCount--;

    pauseMtx.unlock();

}

void AbsMotionController::close()
{

    requestCount++;

    std::unique_lock<std::mutex> lck(pauseMtx);

    isClosed = true;
    isPaused = false;

    pauseCv.notify_all();

    requestCount--;

    pauseMtx.unlock();

    this->mainThread.join();


}

void AbsMotionController::loop()
{
    while(true) {

        std::unique_lock<std::mutex> lck(pauseMtx);

        while(this->isPaused || requestCount>0)
            pauseCv.wait(lck);

        if (isClosed)
            break;


        if(this->nextReadTime < this->nextWriteTime) {

            onRead();

            std::this_thread::sleep_until(this->nextReadTime);

            read();

            afterRead();

            this->nextReadTime += this->readPeriod;

        }else {

            onWrite();

            std::this_thread::sleep_until(this->nextWriteTime);

            write();

            afterWrite();

            this->nextWriteTime += this->writePeriod;
        }

        pauseMtx.unlock();

    }

}


AbsMotionController::AbsMotionController()
{
    //this->jointController = _jointController;

    //fazer a thread

    this->isPaused = true;

    isClosed = false;

    mainThread = std::thread(&AbsMotionController::loop, this);

}
