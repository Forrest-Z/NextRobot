//
// Created by waxz on 6/28/22.
//

#ifndef DOCKER_DEMO_SUSPEND_H
#define DOCKER_DEMO_SUSPEND_H

#include <mutex>
#include <condition_variable>
#include <chrono>

namespace common{

    //https://stackoverflow.com/questions/51817018/can-you-implement-a-timer-without-a-sleep-in-it-using-standard-c-c11-only
    // repeatedly sleeping for small amounts as in your example is a bad idea. This will burn a lot of CPU on needlessly rescheduling and waking threads
    struct Suspend{
        std::mutex mtx;
        std::unique_lock<std::mutex> locker;
        std::condition_variable cv;
//        std::chrono::duration<int, std::ratio<1, 1000>> time_count(1000);
        Suspend():locker(mtx){

        }
        void sleep(int ms){
            cv.wait_for(locker, std::chrono::duration<int, std::ratio<1, 1000>> (ms), [] {  return false;  });

        }
    };
}
#endif //DOCKER_DEMO_SUSPEND_H
