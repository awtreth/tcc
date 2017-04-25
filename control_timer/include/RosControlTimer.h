#ifndef CONTROLLER_TIME_HANDLER_H
#define CONTROLLER_TIME_HANDLER_H

#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <IController.h>
#include <IHardwareInterface.h>

template<typename Controller = IController, typename HardwareInterface = IHardwareInterface>
class ControllerTimeHandler {

private:

    typedef std::shared_ptr<Controller> ControllerPtr;
    typedef std::shared_ptr<HardwareInterface> HardwareInterfacePtr;

    std::thread mainThread;

    HardwareInterfacePtr hardwareInterface;
    std::map<std::string,ControllerPtr> controllers;

    std::chrono::microseconds period = std::chrono::microseconds(20e3);//por padrão: 20ms


    std::chrono::microseconds shiftDuration;

    std::chrono::steady_clock::time_point nextReadTime;

    std::chrono::steady_clock::time_point nextWriteTime;

    double periodShift = 0.5;

    bool isPaused = true;

    bool isClosed = false;

    std::mutex pauseMtx;

    std::condition_variable pauseCv;

    bool updateShiftPeriod();


    static void* loop(void*);

    bool initThread();

    bool startIntervention();
    bool stopIntervention();

    std::chrono::steady_clock::time_point resumeTime;

protected:
    //permite outras implementações
    virtual bool update();//entre a leitura e a escrita
    virtual bool prepareRead();//entre a escrita e a leitura
    virtual bool onReadMiss(std::chrono::steady_clock::time_point desired, std::chrono::steady_clock::time_point realization);
    virtual bool onWriteMiss(std::chrono::steady_clock::time_point desired, std::chrono::steady_clock::time_point realization);
public:

    ControllerTimeHandler();
    ControllerTimeHandler(HardwareInterface* hwInterface);

    bool setHardwareInterface(HardwareInterface* hwInterface);

    bool loadController(std::string controllerName, Controller* controller);
    bool unloadController(std::string controllerName, Controller* controller_out = NULL);
    bool switchController(std::string controller_out_name, std::string controller_in_name, Controller* controller_in, Controller* controller_out = NULL);

    bool setPeriod(const std::chrono::microseconds duration);
    std::chrono::microseconds getPeriod() const;

    bool resumeLoop(std::chrono::microseconds readWaitTime = getPeriod());
    bool pauseLoop();
    bool close();

    bool setFrequency(double freq);
    double getFrequency();

    bool setPeriodShift(double shift);
    double getPeriodShift() const;

};

//#include <src/ControllerTimeHandler.cpp>

#endif
