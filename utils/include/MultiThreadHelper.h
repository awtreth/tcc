#ifndef MULTI_THREAD_HELPER_H
#define MULTI_THREAD_HELPER_H

#include <mutex>

class MultiThreadHelper {

public:

    template<typename ParamType>
    static ParamType getSharedParam(std::mutex& mtx, ParamType& param){

        mtx.lock();

        auto copy = param;
        mtx.unlock();

        return copy;
    }

    template<typename ParamType>
    static void setSharedParam(std::mutex* mtx, ParamType& param, const ParamType& newValue){
        mtx->lock();
        param = newValue;
        mtx->unlock();
    }

};

#endif
