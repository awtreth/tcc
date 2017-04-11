#include <ControllerTimeHandler.h>


template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::updateShiftPeriod()
{
    this->shiftDuration = std::chrono::microseconds( period*this->periodShift);
    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::startIntervention()
{
    pauseMtx.lock();
    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::stopIntervention()
{
    pauseMtx.unlock();
    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::update()
{
    for(auto controller : controllers)
        controller.second.update(std::chrono::steady_clock::now());

    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::prepareRead()
{
    for(auto controller : controllers)
        controller.second->prepareRead(std::chrono::steady_clock::now());

    return true;
}


template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::setPeriodShift(double value)
{
    if(value > 0 && value < 1){

        std::unique_lock<std::mutex> lck(pauseMtx);

        periodShift = value;
        updateShiftPeriod();

        pauseMtx.unlock();

    }
    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::resumeLoop(long readWaitTime)
{

    std::unique_lock<std::mutex> lck(pauseMtx);

    this->nextReadTime = std::chrono::steady_clock::now() + std::chrono::microseconds(readWaitTime);
    this->nextWriteTime = this->nextReadTime + this->shiftDuration;

    this->isPaused = false;

    pauseCv.notify_all();

    pauseMtx.unlock();
    return true;
}


template<typename Controller, typename HardwareInterface>
void *ControllerTimeHandler<Controller,HardwareInterface>::loop(void * param)
{
    ControllerTimeHandler<Controller,HardwareInterface> *sync = (ControllerTimeHandler<Controller,HardwareInterface> *) param;

    while(true) {

        std::unique_lock<std::mutex> lck(sync->pauseMtx);

        while(sync->isPaused)
            sync->pauseCv.wait(lck);

        if (sync->isClosed)
            break;


        sync->prepareRead();

        std::this_thread::sleep_until(sync->nextReadTime);
        sync->hardwareInterface.read();

        sync->update();

        std::this_thread::sleep_until(sync->nextWriteTime);
        sync->hardwareInterface.write();

        sync->nextReadTime += sync->period;
        sync->nextWriteTime += sync->period;

        sync->pauseMtx.unlock();

    }

    return NULL;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::initThread()
{
    this->isPaused = true;

    isClosed = false;

    mainThread = std::thread(&this->loop, this);

    mainThread.detach();

    struct sched_param param;

    param.__sched_priority = 51;

    //TODO: colocar verificacao se a priorida foi configurada mesmo
    pthread_setschedparam(mainThread.native_handle(), SCHED_FIFO, &param);

    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::close()
{
    std::unique_lock<std::mutex> lck(pauseMtx);

    isClosed = true;
    isPaused = false;

    pauseCv.notify_all();

    pauseMtx.unlock();

    if(mainThread.joinable())
        this->mainThread.join();
    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::setPeriod(const std::chrono::microseconds duration)
{
    std::unique_lock<std::mutex> lck(pauseMtx);

    period = duration;

    updateShiftPeriod();

    pauseMtx.unlock();

    return true;
}

template<typename Controller, typename HardwareInterface>
std::chrono::microseconds ControllerTimeHandler<Controller,HardwareInterface>::getPeriod() const
{
    return period;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::pauseLoop()
{
    std::unique_lock<std::mutex> lck(pauseMtx);

    isPaused = true;

    pauseMtx.unlock();
    return true;
}
