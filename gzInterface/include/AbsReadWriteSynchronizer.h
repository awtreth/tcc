#ifndef AbsReadWriteSynchronizer_H
#define AbsReadWriteSynchronizer_H

#include <JointController.h>
#include <Joint.h>
#include <chrono>
#include <thread>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <pthread.h>
#include <time.h>

using namespace std::chrono;

class AbsReadWriteSynchronizer {

    private:

    //JointControllerPtr jointController ;

    std::thread mainThread;

    pthread_t mainPThread;

    std::chrono::microseconds writePeriod;//TODO: colocar valor padrão

    std::chrono::microseconds readPeriod;

    int readWritePeriodRatio = 1;

    double readWriteShift = 0.5;

    std::chrono::microseconds shiftDuration;

    steady_clock::time_point nextReadTime;

    steady_clock::time_point nextWriteTime;

    timespec nextReadTimeTs;

    timespec nextWriteTimeTs;


    bool isPaused = true;

    int requestCount = 0;

    std::mutex pauseMtx;

    std::mutex nextTimeMtx;

    std::condition_variable pauseCv;

    void updateShiftPeriod();

    bool isClosed = false;

    static void* loop(void*);

    protected:

    steady_clock::time_point getNextReadTime() const;

    steady_clock::time_point getNextWriteTime() const;

    virtual void onRead()=0;

    virtual void read()=0;

    virtual void afterRead()=0;

    virtual void onWrite()=0;

    virtual void write()=0;

    virtual void afterWrite()=0;

    virtual void onReadMiss()=0;

    virtual void onWriteMiss()=0;

    void startIntervention();

    void stopIntervention();

    public:

    AbsReadWriteSynchronizer();

    unsigned int getWritePeriod() const;
    void setWritePeriod(unsigned int value);
    unsigned int getReadPeriod() const;
    void setReadPeriod(unsigned int value);
    int getReadWritePeriodRatio() const;
    void setReadWritePeriodRatio(int value);
    double getReadWriteShift() const;
    void setReadWriteShift(double value);

    void resumeLoop(long readWaitTime = 0);

    void pauseLoop();

    void close();
};




#endif
