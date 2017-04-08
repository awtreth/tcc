#ifndef CONTROLLER_TIME_HANDLER_H
#define CONTROLLER_TIME_HANDLER_H

#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable


template<typename Controller, typename HardwareInterface>
class ControllerTimeHandler {

private:

    typedef std::shared_ptr<Controller> ControllerPtr;
    typedef std::shared_ptr<HardwareInterface> HardwareInterfacePtr;

    std::thread mainThread;

    std::chrono::microseconds period;//por padrão: 50ms

    double periodShift = 0.5;

    std::chrono::microseconds shiftDuration;

    std::chrono::steady_clock::time_point nextReadTime;

    std::chrono::steady_clock::time_point nextWriteTime;

    bool isPaused = true;

    int requestCount = 0;

    std::mutex requestCountMtx;

    std::mutex pauseMtx;

    std::mutex nextTimeMtx;

    std::condition_variable pauseCv;

    bool updateShiftPeriod();

    bool isClosed = false;

    static void* loop(void*);

    bool initThread();

    bool startIntervention();

    bool stopIntervention();

    HardwareInterfacePtr hardwareInterface;
    //std::map<std::string,HardwareInterfacePtr> hardwareInterfaces;
    std::map<std::string,ControllerPtr> controllers;


protected:
    //permite outras implementacoes
    virtual bool update();
    virtual bool prepareRead();

public:

    bool close();

    ControllerTimeHandler();
    ControllerTimeHandler(HardwareInterface* hwInterface);//10ms

    bool loadController(std::string controllerName, Controller* controller);
    bool unloadController(std::string controllerName);

    bool setPeriod(const std::chrono::microseconds duration);
    std::chrono::microseconds getPeriod() const;

    bool resumeLoop(long readWaitTime = 0);

    bool pauseLoop();

    bool setFrequency(double freq);
    double getFrequency();

    bool setPeriodShift(double shift);
    double getPeriodShift();

};
#endif
