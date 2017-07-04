#include <ControlTimer.h>
#include <iostream>
#include <functional>
#include <unistd.h>

void ControlTimer::update()
{    
    for(auto controller : controllers)
        controller.second->update(std::chrono::steady_clock::now());
}

void ControlTimer::write()
{
    hardwareInterface->write();
}

void ControlTimer::defaultInit()
{
    setFrequency(50);
    initThread();
}


void ControlTimer::prepareRead()
{
    for(auto controller : controllers)
        controller.second->prepareRead(std::chrono::steady_clock::now());
}

void ControlTimer::onMiss(std::chrono::steady_clock::time_point desire, std::chrono::_V2::steady_clock::time_point realization){

    std::cout << "ON MISS:"
              << "\tDelay="<< std::chrono::duration_cast<std::chrono::microseconds>(realization-desire).count()/1e3 << "ms\t"
              << "\tPeriod="<< period.count()/1e3 << "ms\t"
              << std::endl;

}


ControlTimer::ControlTimer(HardwareInterfacePtr hwInterface)
{
    defaultInit();
    hardwareInterface = hwInterface;
}

ControlTimer::~ControlTimer(){
    this->close();
}


bool ControlTimer::loadController(std::string controllerName, ControllerPtr controller)
{
    //TODO: fazer tratamentos adequados para nao substituir existentes
    std::lock_guard<std::mutex> lck(requestMtx);

    this->controllers[controllerName] = controller;

    return true;
}


ControllerPtr ControlTimer::unloadController(std::string controllerName)
{
    //TODO: fazer tratamentos adequados para retirar um controller

    std::lock_guard<std::mutex> lck(requestMtx);

    auto controller_out = controllers[controllerName];

    controllers.erase(controllerName);

    return controller_out;
}


ControllerPtr ControlTimer::switchController(std::string controller_out_name, std::string controller_in_name, ControllerPtr controller_in)
{
    //TODO: fazer tratamentos adequados do switch
    std::lock_guard<std::mutex> lck(requestMtx);

    auto controller_out = controllers[controller_out_name];

    controllers.erase(controller_out_name);

    controllers[controller_in_name] = controller_in;

    return controller_out;
}


bool ControlTimer::resumeLoop(std::chrono::microseconds readWaitTime)
{

    std::unique_lock<std::mutex> lck(requestMtx);

    if(this->hardwareInterface){
        this->resumeTime = std::chrono::steady_clock::now();

        this->nextLoopTime = this->resumeTime + readWaitTime;

        this->isPaused = false;

        pauseCv.notify_all();

        lck.unlock();
        return true;
    }else{
        lck.unlock();
        return false;//TODO: error msg or exception
    }

}




void ControlTimer::loop()
{
    std::chrono::steady_clock::time_point now;

    //Verifica se esta pausado
    std::unique_lock<std::mutex> lck(requestMtx);
    while(isPaused)
        pauseCv.wait(lck);
    lck.unlock();


    while(true) {

        loopUpdateCheck();

        //Sai se o ControlTimer for fechado
        if (isClosed)
            break;

        prepareRead();

        auto now = std::chrono::steady_clock::now();

        std::this_thread::sleep_until(nextLoopTime);

        if(nextLoopTime > resumeTime && now > nextLoopTime){
            onMiss(nextLoopTime,now);
            nextLoopTime = now;
        }

        read();
        update();
        write();

        nextLoopTime += rtPeriod;

    }

    sleep(1);
}

void ControlTimer::loopUpdateCheck()
{
    //Verifica se esta pausado
    std::unique_lock<std::mutex> lck(requestMtx,std::try_to_lock);

    if(lck.owns_lock()){
        while(isPaused)
            pauseCv.wait(lck);

        if(hasPeriodChange){
            rtPeriod = period;
            hasPeriodChange = false;
        }

        if(hasControllerChange){
            rtControllers = controllers;
            hasControllerChange = false;
        }

        lck.unlock();
    }
}

bool ControlTimer::initThread()
{
    this->isPaused = true;

    this->isClosed = false;

    mainThread = std::thread(&ControlTimer::loop, this);

//    mainThread.detach();

    struct sched_param param;

    param.__sched_priority = 51;

    //TODO: success check
    std::cout << pthread_setschedparam(mainThread.native_handle(), SCHED_FIFO, &param) << std::endl;

    return true;
}

void ControlTimer::read()
{
    hardwareInterface->read();
}


void ControlTimer::close()
{
    std::unique_lock<std::mutex> lck(requestMtx);

    isClosed = true;
    isPaused = false;

    pauseCv.notify_all();

    lck.unlock();

    if(mainThread.joinable())
        this->mainThread.join();

}


bool ControlTimer::setFrequency(double freq)
{
    //TODO: exception?
    if(freq<=0)
        return false;

    std::lock_guard<std::mutex> lck(requestMtx);

    hasPeriodChange = true;
    period = std::chrono::microseconds(long((1/freq)*1e6));

    return true;

}


double ControlTimer::getFrequency()
{
    return (1./(period.count()/double(1e6)));
}


void ControlTimer::pauseLoop()
{
    std::lock_guard<std::mutex> lck(requestMtx);

    isPaused = true;

}
