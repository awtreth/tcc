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
#include <memory>
#include <iostream>

using namespace std;

class ControlTimer {

private:

    thread mainThread;

    HardwareInterfacePtr hardwareInterface;
    map<string,ControllerPtr> controllers;

    chrono::microseconds period = chrono::microseconds(long(20e3));//por padrão: 20ms

    chrono::microseconds shiftDuration;

    chrono::steady_clock::time_point nextReadTime;

    chrono::steady_clock::time_point nextWriteTime;

    double periodShift = 0.5;

    bool isPaused = true;

    bool isClosed = false;

    mutex pauseMtx;

    condition_variable pauseCv;

    bool updateShiftPeriod();

    void loop();

    bool initThread();

    chrono::steady_clock::time_point resumeTime;

    bool update();

    void defaultInit();

protected:

    virtual bool prepareRead();//entre a escrita e a leitura
    virtual bool onReadMiss(chrono::steady_clock::time_point desired, chrono::steady_clock::time_point realization);
    virtual bool onWriteMiss(chrono::steady_clock::time_point desired, chrono::steady_clock::time_point realization);

public:

    ControlTimer();
    ControlTimer(HardwareInterfacePtr hwInterface);

    virtual ~ControlTimer(){std::cout << "DESTRUCTOR" << std::endl;}

    bool setHardwareInterface(HardwareInterfacePtr hwInterface);

    //TODO: check if it works
    bool loadController(string controllerName, ControllerPtr controller);
    ControllerPtr unloadController(string controllerName);
    ControllerPtr switchController(string controller_out_name, string controller_in_name, ControllerPtr controller_in);

    bool setPeriod(const chrono::microseconds duration);
    chrono::microseconds getPeriod() const;

    bool resumeLoop(chrono::microseconds readWaitTime = chrono::microseconds(0));
    bool pauseLoop();
    bool close();

    bool setFrequency(double freq);
    double getFrequency();

    bool setPeriodShift(double shift);
    double getPeriodShift() const;

};

#endif
