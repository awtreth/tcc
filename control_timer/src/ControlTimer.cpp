#include <ControllerTimeHandler.h>

#include <iostream>

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
        controller.second->update(std::chrono::steady_clock::now());

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
bool ControllerTimeHandler<Controller,HardwareInterface>::onReadMiss(std::chrono::_V2::steady_clock::time_point desired, std::chrono::_V2::steady_clock::time_point realization)
{
    //FIXME: cout nao me parece adequado para implementacao padrao, pois gera atraso
    //TODO: evitar repeticao de codigo com onWriteMiss
    std::cout << "READ MISS:"
              << "\tDelay="<< std::chrono::duration_cast<std::chrono::microseconds>(realization-desired).count()/1e3 << "ms\t"
              << "\tPeriod="<< period.count()/1e3 << "ms\t"
              << std::endl;

    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::onWriteMiss(std::chrono::_V2::steady_clock::time_point desired, std::chrono::_V2::steady_clock::time_point realization)
{
    //FIXME: cout nao me parece adequado para implementacao padrao, pois gera atraso
    //TODO: evitar repeticao de codigo com onReadMiss
    std::cout << "WRITE MISS:"
              << "\tDelay="<< std::chrono::duration_cast<std::chrono::microseconds>(realization-desired).count()/1e3 << "ms\t"
              << "\tPeriod="<< period.count()/1e3 << "ms\t"
              << std::endl;
    return true;
}

template<typename Controller, typename HardwareInterface>
ControllerTimeHandler<Controller, HardwareInterface>::ControllerTimeHandler()
{
    periodShift = 0.5;
    period = std::chrono::milliseconds(20);
    updateShiftPeriod();
    initThread();
}

template<typename Controller, typename HardwareInterface>
ControllerTimeHandler<Controller, HardwareInterface>::ControllerTimeHandler(HardwareInterface *hwInterface)
{
    ControllerTimeHandler<Controller,HardwareInterface>();
    setHardwareInterface(hwInterface);
    resumeLoop();
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller, HardwareInterface>::setHardwareInterface(HardwareInterface *hwInterface)
{
    startIntervention();
    this->hardwareInterface = HardwareInterfacePtr(hwInterface);
    stopIntervention();

    return this->hardwareInterface;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::loadController(std::__cxx11::string controllerName, Controller *controller)
{
    //TODO: fazer tratamentos adequados para nao substituir existentes
    startIntervention();
    this->controllers[controllerName] = ControllerPtr(controller);
    stopIntervention();

    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::unloadController(std::__cxx11::string controllerName, Controller *controller_out)
{
    //TODO: fazer tratamentos adequados para retirar um controller
    startIntervention();

    if(controller_out != NULL)
        controller_out = controllers[controllerName].get();

    controllers.erase(controllerName);

    stopIntervention();

    return true;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::switchController(std::__cxx11::string controller_out_name, std::__cxx11::string controller_in_name, Controller *controller_in, Controller *controller_out)
{
    //TODO: fazer tratamentos adequados do switch

    startIntervention();

    if(controller_out != NULL)
        controller_out = controllers[controller_out_name].get();

    controllers.erase(controller_out_name);

    this->controllers[controller_in_name] = ControllerPtr(controller_in);

    stopIntervention();

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
double ControllerTimeHandler<Controller,HardwareInterface>::getPeriodShift() const
{
    return periodShift;
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::resumeLoop(std::chrono::microseconds readWaitTime)
{

    std::unique_lock<std::mutex> lck(pauseMtx);

    if(this->HardwareInterfacePtr){
        this->nextReadTime = std::chrono::steady_clock::now() + readWaitTime;
        this->nextWriteTime = this->nextReadTime + this->shiftDuration;

        this->isPaused = false;
        this->resumeTime = nextReadTime;

        pauseCv.notify_all();
        pauseMtx.unlock();
        return true;
    }else{
        pauseMtx.unlock();
        return false;//TODO: error msg or exception
    }

}


template<typename Controller, typename HardwareInterface>
void *ControllerTimeHandler<Controller,HardwareInterface>::loop(void * param)
{
    ControllerTimeHandler<Controller,HardwareInterface> *sync = (ControllerTimeHandler<Controller,HardwareInterface> *) param;

    bool evaluateMiss = true;

    std::chrono::steady_clock::time_point now;

    while(true) {

        std::unique_lock<std::mutex> lck(sync->pauseMtx);

        while(sync->isPaused)
            sync->pauseCv.wait(lck);

        if (sync->isClosed)
            break;

        sync->prepareRead();

        if(sync->nextReadTime < sync->resumeTime)
            evaluateMiss=false;
        else
            evaluateMiss=true;

        now = std::chrono::steady_clock::now();

        std::this_thread::sleep_until(sync->nextReadTime);
        sync->hardwareInterface->read();

        if(evaluateMiss && now > sync->nextReadTime)
            onReadMiss(sync->nextReadTime, now);

        sync->update();

        now = std::chrono::steady_clock::now();

        std::this_thread::sleep_until(sync->nextWriteTime);
        sync->hardwareInterface->write();

        if(evaluateMiss && now > sync->nextWriteTime)
            onWriteMiss(sync->nextWriteTime, now);

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
bool ControllerTimeHandler<Controller,HardwareInterface>::setFrequency(double freq)
{
    //TODO: exception?
    if(freq<=0)
        return false;

    setPeriod(std::chrono::microseconds((1/freq)*1e6));

    return true;

}

template<typename Controller, typename HardwareInterface>
double ControllerTimeHandler<Controller,HardwareInterface>::getFrequency()
{
    return (1./(period.count()/(double)1e6));
}

template<typename Controller, typename HardwareInterface>
bool ControllerTimeHandler<Controller,HardwareInterface>::setPeriod(const std::chrono::microseconds duration)
{
    if(duration.count()<=0)
        return false;

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
