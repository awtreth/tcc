#include <control_loop.h>
#include <iostream>
#include <functional>
#include <unistd.h>

using namespace control_loop;

void ControlLoop::update()
{    
    for(auto controller : controllers)
        controller.second->update(std::chrono::steady_clock::now());
}

void ControlLoop::write()
{
    hardwareInterface->write();
}

void ControlLoop::defaultInit()
{
    setFrequency(50);
    initThread();
}


void ControlLoop::prepareRead()
{
    for(auto controller : controllers)
        controller.second->prepareRead(std::chrono::steady_clock::now());
}

void ControlLoop::onMiss(std::chrono::steady_clock::time_point desire, std::chrono::_V2::steady_clock::time_point realization){

    std::cout << "ON MISS:"
              << "\tDelay="<< std::chrono::duration_cast<std::chrono::microseconds>(realization-desire).count()/1e3 << "ms\t"
              << "\tPeriod="<< period.count()/1e3 << "ms\t"
              << std::endl;

}


//ControlLoop::ControlLoop()
//{
//    defaultInit();
//}

ControlLoop::ControlLoop(HardwarePtr hwInterface)
{
    defaultInit();
    hardwareInterface = hwInterface;
}

ControlLoop::~ControlLoop(){
    this->close();
}


bool ControlLoop::loadController(std::string controllerName, ControllerPtr controller)
{
    //TODO: fazer tratamentos adequados para nao substituir existentes
    std::lock_guard<std::mutex> lck(requestMtx);

    this->controllers[controllerName] = controller;

    return true;
}


ControllerPtr ControlLoop::unloadController(std::string controllerName)
{
    //TODO: fazer tratamentos adequados para retirar um controller

    std::lock_guard<std::mutex> lck(requestMtx);

    auto controller_out = controllers[controllerName];

    controllers.erase(controllerName);

    return controller_out;
}


ControllerPtr ControlLoop::switchController(std::string controller_out_name, std::string controller_in_name, ControllerPtr controller_in)
{
    //TODO: fazer tratamentos adequados do switch
    std::lock_guard<std::mutex> lck(requestMtx);

    auto controller_out = controllers[controller_out_name];

    controllers.erase(controller_out_name);

    controllers[controller_in_name] = controller_in;

    return controller_out;
}


bool ControlLoop::resumeLoop(std::chrono::microseconds readWaitTime)
{

    std::unique_lock<std::mutex> lck(requestMtx);

    //The loop is resumed just when there is a hardware loaded
    if(this->hardwareInterface){

        this->resumeTime = std::chrono::steady_clock::now();

        //Sets the first iterations one period from the resume time
        this->nextLoopTime = this->resumeTime + readWaitTime;

        this->isPaused = false;

        //Notify the main thread
        pauseCv.notify_all();

        lck.unlock();
        return true;
    }else{
        lck.unlock();
        return false;//TODO: error msg or exception
    }

}




void ControlLoop::loop()
{
    std::chrono::steady_clock::time_point now;

    //Check if the loop is paused
    std::unique_lock<std::mutex> lck(requestMtx);
    while(isPaused)
        pauseCv.wait(lck);
    lck.unlock();


    while(true) {

        //Process external commands, such as frequency update
        loopUpdateCheck();

        if (isClosed)
            break;

        //Right after the last write()
        prepareRead();

        auto now = std::chrono::steady_clock::now();

        std::this_thread::sleep_until(nextLoopTime);

        //Without the first condition, the method could be called on the first iteration
        if(nextLoopTime > resumeTime && now > nextLoopTime){
            onMiss(nextLoopTime,now);
            nextLoopTime = now;//update the current deadline
        }

        read();
        update();
        write();

        nextLoopTime += rtPeriod;

    }

    //This command is needed to give time to close() method to join this thread
    sleep(1);
}

void ControlLoop::loopUpdateCheck()
{

    //try_lock is used to be real-time safe, otherwise it could depend on other non-realtime threads
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

bool ControlLoop::initThread()
{
    this->isPaused = true;

    this->isClosed = false;

    mainThread = std::thread(&ControlLoop::loop, this);

//    mainThread.detach();

    //TODO:: make scheduler policy and priority settable on construction

    struct sched_param param;

    //It can't be greater than 98
    param.__sched_priority = 98;

    //TODO: success check
    std::cout << pthread_setschedparam(mainThread.native_handle(), SCHED_FIFO, &param) << std::endl;

    return true;
}

void ControlLoop::read()
{
    hardwareInterface->read();
}


void ControlLoop::close()
{
    std::unique_lock<std::mutex> lck(requestMtx);

    isClosed = true;
    isPaused = false;

    //Unpause loop
    pauseCv.notify_all();

    lck.unlock();

    //Wait mainThread to finish
    if(mainThread.joinable())
        this->mainThread.join();

}


bool ControlLoop::setFrequency(double freq)
{
    //TODO: exception?
    if(freq<=0)
        return false;

    std::lock_guard<std::mutex> lck(requestMtx);

    hasPeriodChange = true;
    period = std::chrono::microseconds(long((1/freq)*1e6));

    return true;

}


double ControlLoop::getFrequency()
{
    return (1./(period.count()/double(1e6)));
}


void ControlLoop::pauseLoop()
{
    std::lock_guard<std::mutex> lck(requestMtx);

    isPaused = true;

}
