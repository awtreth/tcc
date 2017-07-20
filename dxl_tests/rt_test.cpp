#include <iostream>
#include <cstdio>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <cstring>
#include <vector>
#include <thread>

void help() {
    std::cout << "\nrt_test [sleep_time] [n_iterations] [rt_option]" << "\n\n";

    std::cout << "sleep_time: default 1000 [us]" << std::endl;

    std::cout << "n_iterations: default 1000" << std::endl;

    std::cout << "rt_option: '-rt' or '-nonrt'" << std::endl;
}

int main(int argc, char** argv){

    long sleep_time = 1000; //us
    long n_iterations = 1000;
    bool real_time = false;
    bool show_help = false;

    if(argc >= 2){

        if(std::string(argv[1]).find("help")!=std::string::npos)
            show_help = true;
        else {
            sleep_time = std::stol(argv[1]);

            if(argc >= 4)
                n_iterations = std::stol(argv[2]);

            if(argc >= 4){
                if(strcmp(argv[3],"-rt") == 0)
                    real_time = true;
                else if(strcmp(argv[3],"-nonrt") == 0)
                    real_time = false;
                else
                    show_help = true;
            }else if(argc > 4)
                show_help = true;
        }
    }

    if(show_help){
        help();
        exit(-1);
    }

    if(real_time){
        mlockall(MCL_FUTURE);

        struct sched_param param;
        //param.__sched_priority = 98;
        param.__sched_priority = 31;

        //if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param)){
        if(pthread_setschedparam(pthread_self(), SCHED_RR, &param)){
            std::cout << "No permission to set real_time scheduler" << std::endl;
            exit(-1);
        }

    }

    std::vector<std::chrono::microseconds> timestamps(static_cast<unsigned long>(n_iterations));

    std::chrono::microseconds sleep_duration(sleep_time);

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now()+std::chrono::seconds(1);
    std::chrono::steady_clock::time_point next_time = start_time + sleep_duration;
    std::chrono::steady_clock::time_point prev_time = start_time;

    std::this_thread::sleep_until(start_time);

    for (ulong i = 0; i < timestamps.size(); i++){

        std::this_thread::sleep_until(next_time);
        timestamps[i] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-prev_time);
        prev_time = next_time;
        next_time += sleep_duration;
    }

    if(real_time)
        munlockall();

    std::cout << "expected_period," << sleep_time << std::endl;
    std::cout << "iteration,period" << std::endl;

    for(ulong i = 0; i < timestamps.size(); i++)
        std::cout << i << "," << timestamps[i].count() << std::endl;

    return 0;

}
