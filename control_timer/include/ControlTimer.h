#ifndef CONTROLLER_TIME_HANDLER_H
#define CONTROLLER_TIME_HANDLER_H

#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>              // mutex, unique_lock
#include <condition_variable> // condition_variable
#include <IController.h>
#include <IHardwareInterface.h>
#include <functional>

class ControlTimer {

private:

    std::thread mainThread;

    HardwareInterfacePtr hardwareInterface;
    std::map<std::string,ControllerPtr> controllers;
    std::map<std::string,ControllerPtr> rtControllers;

    std::chrono::microseconds period = std::chrono::microseconds(long(20e3));//por padrão: 20ms
    std::chrono::microseconds rtPeriod;

    //Variable access control flags
    bool isPaused = true;
    bool isClosed = false;

    bool hasPeriodChange = true;
    bool hasControllerChange = true;

    std::mutex requestMtx;

    std::condition_variable pauseCv;

    std::chrono::steady_clock::time_point resumeTime;
    std::chrono::steady_clock::time_point nextLoopTime;

    //Initialization helper methods
    void defaultInit();
    bool initThread();

    void loop();
    void loopUpdateCheck();//return false if it was closed

    bool setHardwareInterface(HardwareInterfacePtr hwInterface);

protected:

    virtual void prepareRead();//entre a escrita e a leitura

    virtual void read();
    virtual void update();
    virtual void write();

    virtual void onMiss(std::chrono::steady_clock::time_point desired, std::chrono::steady_clock::time_point realization);

public:

//    ControlTimer();
    ControlTimer(HardwareInterfacePtr hwInterface);

    virtual ~ControlTimer();


    //TODO: check if it works
    bool loadController(std::string controllerName, ControllerPtr controller);
    ControllerPtr unloadController(std::string controllerName);
    ControllerPtr switchController(std::string controller_out_name, std::string controller_in_name, ControllerPtr controller_in);

    bool resumeLoop(std::chrono::microseconds readWaitTime = std::chrono::microseconds(0));
    void pauseLoop();
    void close();

    bool setFrequency(double freq);
    double getFrequency();

};

#endif
