#include <AsyncLogger.h>

void AsyncLogger::logTFunc(){

    int semValue = 0;

    sem_getvalue(&sem,&semValue);

    while(isOn || semValue > 0){
        sem_wait(&sem);
        mtx.lock();

        if(strQueue.size()>0){
            auto str = strQueue.front();
            strQueue.pop();
            mtx.unlock();
            std::cout << str << std::endl;
        }else
            mtx.unlock();
        sem_getvalue(&sem,&semValue);
    }
    sem_destroy(&sem);
}

AsyncLogger::AsyncLogger(){
    reset();
}

void AsyncLogger::reset(){

    strQueue = std::queue<std::string>();

    logThread = std::thread(&AsyncLogger::logTFunc, this);

    sem_init(&sem,0,0);

    isOn = true;
}

void AsyncLogger::write(std::__cxx11::string str){
    mtx.lock();
    strQueue.push(str);
    sem_post(&sem);
    mtx.unlock();
}

void AsyncLogger::close(){
    isOn = false;
    sem_post(&sem);
    logThread.join();
}
