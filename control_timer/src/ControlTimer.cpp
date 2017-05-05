#include <ControlTimer.h>
#include <iostream>
#include <functional>

void ControlTimer::updateShiftDuration()
{
    this->shiftDuration = std::chrono::microseconds(long(period.count()*this->periodShift));
}


bool ControlTimer::update()
{    

    for(auto controller : controllers)
        controller.second->update(std::chrono::steady_clock::now());

    return true;
}

void ControlTimer::write()
{
    hardwareInterface->write();
}

void ControlTimer::defaultInit()
{
    setPeriod(std::chrono::milliseconds(20));
    setPeriodShift(0.5);
    updateShiftDuration();
    initThread();
}


bool ControlTimer::prepareRead()
{
    for(auto controller : controllers)
        controller.second->prepareRead(std::chrono::steady_clock::now());

    return true;
}


bool ControlTimer::onReadMiss(std::chrono::_V2::steady_clock::time_point desired, std::chrono::_V2::steady_clock::time_point realization)
{
    //FIXME: cout nao me parece adequado para implementacao padrao, pois gera atraso
    //TODO: evitar repeticao de codigo com onWriteMiss
    std::cout << "READ MISS:"
              << "\tDelay="<< std::chrono::duration_cast<std::chrono::microseconds>(realization-desired).count()/1e3 << "ms\t"
              << "\tPeriod="<< period.count()/1e3 << "ms\t"
              << std::endl;

    return true;
}


bool ControlTimer::onWriteMiss(std::chrono::_V2::steady_clock::time_point desired, std::chrono::_V2::steady_clock::time_point realization)
{
    //FIXME: cout nao me parece adequado para implementacao padrao, pois gera atraso
    //TODO: evitar repeticao de codigo com onReadMiss
    std::cout << "WRITE MISS:"
              << "\tDelay="<< std::chrono::duration_cast<std::chrono::microseconds>(realization-desired).count()/1e3 << "ms\t"
              << "\tPeriod="<< period.count()/1e3 << "ms\t"
              << std::endl;
    return true;
}


ControlTimer::ControlTimer()
{
    defaultInit();
}


ControlTimer::ControlTimer(HardwareInterfacePtr hwInterface)
{
    defaultInit();
    setHardwareInterface(hwInterface);
    resumeLoop();
}


bool ControlTimer::setHardwareInterface(HardwareInterfacePtr hwInterface)
{
//    std::unique_lock<std::mutex> lck(this->pauseMtx);
    pauseMtx.lock();
    //this->hardwareInterface = HardwareInterfacePtr(hwInterface);
    hardwareInterface = hwInterface;
    pauseMtx.unlock();

    return this->hardwareInterface.operator bool();
}


bool ControlTimer::loadController(std::string controllerName, ControllerPtr controller)
{
    //TODO: fazer tratamentos adequados para nao substituir existentes
//    std::unique_lock<std::mutex> lck(this->pauseMtx);
    pauseMtx.lock();
    //this->controllers[controllerName] = ControllerPtr(controller);
    this->controllers[controllerName] = controller;
    pauseMtx.unlock();

    return true;
}


ControllerPtr ControlTimer::unloadController(std::string controllerName)
{
    //TODO: fazer tratamentos adequados para retirar um controller
//    std::unique_lock<std::mutex> lck(this->pauseMtx);
    pauseMtx.lock();
    auto controller_out = controllers[controllerName];

    controllers.erase(controllerName);

    pauseMtx.unlock();

    return controller_out;
}


ControllerPtr ControlTimer::switchController(std::string controller_out_name, std::string controller_in_name, ControllerPtr controller_in)
{
    //TODO: fazer tratamentos adequados do switch
//    std::unique_lock<std::mutex> lck(this->pauseMtx);
    pauseMtx.lock();
    auto controller_out = controllers[controller_out_name];

    controllers.erase(controller_out_name);

    //controllers[controller_in_name] = ControllerPtr(controller_in);
    controllers[controller_in_name] = controller_in;

    pauseMtx.unlock();

    return controller_out;
}



bool ControlTimer::setPeriodShift(double value)
{
    if(value > 0 && value < 1){

        pauseLoop();
        periodShift = value;
        updateShiftDuration();
        return resumeLoop();
    }

    return false;
}


double ControlTimer::getPeriodShift() const
{
    return periodShift;
}


bool ControlTimer::resumeLoop(std::chrono::microseconds readWaitTime)
{

    std::lock_guard<std::mutex> lck(this->pauseMtx);

    if(this->hardwareInterface){
        this->resumeTime = std::chrono::steady_clock::now();

        this->nextReadTime = this->resumeTime + readWaitTime;
        this->nextWriteTime = this->nextReadTime + this->shiftDuration;

        this->isPaused = false;

        this->pauseCv.notify_all();
        this->pauseMtx.unlock();
        return true;
    }else{
        pauseMtx.unlock();
        return false;//TODO: error msg or exception
    }

}


void ControlTimer::loop()
{
    bool evaluateMiss = true;

    std::chrono::steady_clock::time_point now;

    while(true) {


        //Verifica se esta pausado
        std::unique_lock<std::mutex> lck(pauseMtx);

        while(isPaused)
            pauseCv.wait(lck);

        //Sai se o ControlTimer for fechado
        if (isClosed)
            break;

        //Para evitar Miss na primeira iteracao
        if(nextReadTime <= resumeTime)
            evaluateMiss=false;
        else
            evaluateMiss=true;


        //Rotina a ser executada antes da leitura
        prepareRead();

        using namespace std::placeholders;
        //read
        onReadWrite(evaluateMiss,nextReadTime,std::bind(&ControlTimer::read,this),std::bind(&ControlTimer::onReadMiss,this,_1,_2));

        update();

        //write;
        onReadWrite(evaluateMiss,nextWriteTime,std::bind(&ControlTimer::write,this),std::bind(&ControlTimer::onWriteMiss,this,_1,_2));

        nextReadTime += period;
        nextWriteTime += period;

        pauseMtx.unlock();

    }

    //    return NULL;
}

void ControlTimer::onReadWrite(bool evaluateMiss, chrono::steady_clock::time_point timePoint, std::function<void()> readWrite,
                               std::function<bool (chrono::steady_clock::time_point,chrono::steady_clock::time_point)> onMiss)
{
    auto now = std::chrono::steady_clock::now();

    std::this_thread::sleep_until(timePoint);

    readWrite();

    if(evaluateMiss && now > timePoint)
        onMiss(timePoint, now);
}


bool ControlTimer::initThread()
{
    this->isPaused = true;

    this->isClosed = false;

    mainThread = std::thread(&ControlTimer::loop, this);

    mainThread.detach();

        struct sched_param param;

        param.__sched_priority = 51;

        //TODO: colocar verificacao se a priorida foi configurada mesmo
        pthread_setschedparam(mainThread.native_handle(), SCHED_FIFO, &param);

    return true;
}

void ControlTimer::read()
{
    hardwareInterface->read();
}


bool ControlTimer::close()
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


bool ControlTimer::setFrequency(double freq)
{
    //TODO: exception?
    if(freq<=0)
        return false;

    setPeriod(std::chrono::microseconds(long((1/freq)*1e6)));

    return true;

}


double ControlTimer::getFrequency()
{
    return (1./(period.count()/double(1e6)));
}


bool ControlTimer::setPeriod(const std::chrono::microseconds duration)
{
    if(duration.count()<=0)
        return false;

    //    std::unique_lock<std::mutex> lck(pauseMtx);
    pauseLoop();

    period = duration;
    updateShiftDuration();

    return resumeLoop();
}


std::chrono::microseconds ControlTimer::getPeriod() const
{
    return period;
}


bool ControlTimer::pauseLoop()
{
    std::lock_guard<std::mutex> lck(pauseMtx);

    isPaused = true;

    pauseMtx.unlock();
    return true;
}
