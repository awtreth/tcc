#ifndef DUMMY_MOTION_CONTROLLER_H
#define DUMMY_MOTION_CONTROLLER_H

#include <AbsReadWriteSynchronizer.h>
#include <JointController.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <vector>

#include <AsyncLogger.h>

struct SyncLog {
    long period;
    long ts;
    long expected;
    long deviation;
    long tsDeviation;
    double error;
};


class DummyReadWriteSynchronizer : public AbsReadWriteSynchronizer {

    public:

    DummyReadWriteSynchronizer():AbsReadWriteSynchronizer(){
        readLog = std::vector<SyncLog>();
        writeLog = std::vector<SyncLog>();
    }

    std::vector<SyncLog> readLog;
    std::vector<SyncLog> writeLog;

    //AsyncLogger logger;

    ~DummyReadWriteSynchronizer(){
        //logger.close();
    }

    private:

    steady_clock::time_point lastWritePoint = std::chrono::steady_clock::now();

    steady_clock::time_point lastReadPoint = std::chrono::steady_clock::now();

    steady_clock::time_point now;

    // AbsReadWriteSynchronizer interface
    protected:

    virtual void onRead()
    {
//        std::cout << "onRead" << std::endl;
    }

    virtual void read()
    {
        now = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(now-lastReadPoint).count();
        SyncLog log;
        log.period = diff;
        log.deviation = (diff-getReadPeriod());
        log.error = (diff-getReadPeriod())/((double)getReadPeriod());
        log.tsDeviation = std::chrono::duration_cast<std::chrono::microseconds>(now-getNextReadTime()).count();
        log.ts = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        log.expected = std::chrono::duration_cast<std::chrono::microseconds>(getNextReadTime().time_since_epoch()).count();


        readLog.push_back(log);

//        logger.write("read: " + std::to_string(log.period) + " " + std::to_string(log.tsDeviation)
//                      + " " + std::to_string(log.deviation)  + " " + std::to_string(log.error*100) + "%");

        lastReadPoint = now;
    }

    virtual void afterRead()
    {
        //std::cout << "afterRead" << std::endl;
    }

    virtual void onWrite()
    {
//        std::cout << "onWrite" << std::endl;
    }

    virtual void write()
    {
        now = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(now-lastWritePoint).count();

        SyncLog log;
        log.period = diff;
        log.deviation = (diff-getWritePeriod());
        log.error = (diff-getWritePeriod())/((double)getWritePeriod());
        log.tsDeviation = std::chrono::duration_cast<std::chrono::microseconds>(now-getNextWriteTime()).count();
        log.ts = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        log.expected = std::chrono::duration_cast<std::chrono::microseconds>(getNextWriteTime().time_since_epoch()).count();


        writeLog.push_back(log);

//        logger.write("read: " + std::to_string(log.period) + " " + std::to_string(log.tsDeviation)
//                      + " " + std::to_string(log.deviation)  + " " + std::to_string(log.error*100) + "%");
        lastWritePoint = now;
    }

    virtual void afterWrite()
    {
        //std::cout << "afterWrite" << std::endl;
    }

    virtual void onReadMiss()
    {
        std::cout << "onReadMiss" << std::endl;
    }

    virtual void onWriteMiss()
    {
        std::cout << "onWriteMiss" << std::endl;
    }
};




#endif
