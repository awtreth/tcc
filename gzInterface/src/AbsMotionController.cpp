#include "AbsMotionController.h"
#include <thread>
#include <chrono>
#include <math.h>
#include <stdlib.h>
#include <algorithm>

using namespace std::chrono;

unsigned int AbsMotionController::getWritePeriod() const
{
    return writePeriod.count();
}

void AbsMotionController::setWritePeriod(unsigned value)
{
    writePeriod = std::chrono::microseconds(value);
}

unsigned int AbsMotionController::getReadPeriod() const
{
    return readPeriod.count();
}

void AbsMotionController::setReadPeriod(unsigned value)
{
    readPeriod = std::chrono::microseconds(value);
}

int AbsMotionController::getReadWritePeriodRatio() const
{
    return readWritePeriodRatio;
}

void AbsMotionController::setReadWritePeriodRatio(int value)
{
    readWritePeriodRatio = pow(2,value);


}

double AbsMotionController::getReadWriteShift() const
{
    return readWriteShift;
}

void AbsMotionController::setReadWriteShift(double value)
{
    if(value > 0 && value < 1){
        readWriteShift = value;
        this->shiftDuration = std::chrono::microseconds( (unsigned) (std::min(this->readPeriod.count(),this->writePeriod.count())*this->readWriteShift));
    }
}

void AbsMotionController::resume(unsigned int readWaitTime)
{

    this->nextReadTime = std::chrono::steady_clock::now() + std::chrono::microseconds(readWaitTime);
    this->nextWriteTime = this->nextReadTime + this->shiftDuration;

    //liberar loop
}

void AbsMotionController::pause()
{

}

void AbsMotionController::loop()
{
    while(true) {

        if(this->nextReadTime < this->nextWriteTime) {
            std::this_thread::sleep_until(this->nextReadTime);

            //onRead

            this->jointController->readJointStates();

            this->afterRead();

            this->nextReadTime += this->readPeriod;

        }else {
            std::vector<JointCommandPtr> cmds = this->onWrite();

            std::this_thread::sleep_until(this->nextWriteTime);

            this->jointController->sendCommand(cmds);

            this->nextWriteTime += this->writePeriod;
        }
    }

}

AbsMotionController::AbsMotionController(JointControllerPtr _jointController)
{
    this->jointController = _jointController;

    //fazer a thread

    mainThread = std::thread(&AbsMotionController::loop, this);

}



/*
 *Thread Function:
 *
 *
 *
 *
 *
 * if WRITE
 * onwrite
 * write
 *
 * else if READ
 * read
 * ondread
 *
 *
 *
 */
