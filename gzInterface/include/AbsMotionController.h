#ifndef ABSMOTIONCONTROLLER_H
#define ABSMOTIONCONTROLLER_H

#include <JointController.h>
#include <Joint.h>
#include <chrono>
#include <thread>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

using namespace std::chrono;

class AbsMotionController {

    private:

    //JointControllerPtr jointController ;

    std::thread mainThread;

    std::chrono::microseconds writePeriod;//TODO: colocar valor padrão

    std::chrono::microseconds readPeriod;

    int readWritePeriodRatio = 1;

    double readWriteShift = 0.5;

    std::chrono::microseconds shiftDuration;

    steady_clock::time_point nextReadTime;

    steady_clock::time_point nextWriteTime;

    void loop();

    bool isPaused = true;

    int requestCount = 0;

    std::mutex pauseMtx;

    std::mutex nextTimeMtx;

    std::condition_variable pauseCv;

    void updateShiftPeriod();

    bool isClosed = false;

    protected:

    virtual void onRead(){}

    virtual void read(){}

    virtual void afterRead(){}

    virtual void onWrite(){}

    virtual void write(){}

    virtual void afterWrite(){}

    virtual void onReadMiss(){}

    virtual void onWriteMiss(){}

    void startIntervention();

    void stopIntervention();

    public:

    AbsMotionController();

    unsigned int getWritePeriod() const;
    void setWritePeriod(unsigned int value);
    unsigned int getReadPeriod() const;
    void setReadPeriod(unsigned int value);
    int getReadWritePeriodRatio() const;
    void setReadWritePeriodRatio(int value);
    double getReadWriteShift() const;
    void setReadWriteShift(double value);

    void resume(unsigned int readWaitTime);

    void pause();

    void close();
};




#endif
