#ifndef ASYNC_LOGGER_H
#define ASYNC_LOGGER_H

#include <string>
#include <queue>
#include <thread>
#include <semaphore.h>
#include <mutex>
#include <iostream>
#include <memory>

class AsyncLogger {

    private:

    std::queue<std::string> strQueue;

    std::thread logThread;

    sem_t sem;

    std::mutex mtx;

    bool isOn = true;


    void logTFunc();

    public:

    AsyncLogger();

    void reset();

    void write(std::string str);

    void close();


};




#endif

