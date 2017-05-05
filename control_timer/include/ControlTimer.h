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
#include <functional>

using namespace std;

class ControlTimer {

private:

    thread mainThread;

    HardwareInterfacePtr hardwareInterface;
    map<string,ControllerPtr> controllers;

    chrono::microseconds period = chrono::microseconds(long(20e3));//por padrão: 20ms
    double periodShift = 0.5;
    chrono::microseconds shiftDuration;

    chrono::steady_clock::time_point resumeTime;

    bool hasUpdate = true;
    bool isPaused = true;
    bool isClosed = false;

    mutex pauseMtx;
    condition_variable pauseCv;


    chrono::steady_clock::time_point nextReadTime;
    chrono::steady_clock::time_point nextWriteTime;

    //Generic helper methods
    void updateShiftDuration();

    //Initialization helper methods
    void defaultInit();
    bool initThread();

    void read();
    bool update();
    void write();

    void loop();
    void onReadWrite(bool evaluateMiss, chrono::steady_clock::time_point timePoint, std::function<void()> readWrite,
                     std::function<bool (chrono::steady_clock::time_point,chrono::steady_clock::time_point)> onMiss);
    void onWrite(bool evaluateMiss);


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
