//
// Created by waxz on 22-11-14.
//

#ifndef SCAN_REPUBLISHER_TASK_H
#define SCAN_REPUBLISHER_TASK_H

#include <functional>
#include <vector>
#include <algorithm>

#include "clock_time.h"

namespace common{
    struct TaskManager{
        struct Task{
            bool valid = true;
            common::Time time;
            double delay_ms = 100;
            std::function<bool()> func;
            Task(const std::function<bool()>& func_,float delay_ms_ = 100):time(common::FromUnixNow()),delay_ms(delay_ms_),func(func_){
            }
            Task(const Task& rhv){
                valid = rhv.valid;
                time = rhv.time;
                delay_ms = rhv.delay_ms;
                func = std::move(rhv.func);
            }

        };
        std::vector<Task> task_queue;

        TaskManager(){}

        /*
         const std::function<bool()>& func
         return true, keep running at given rate
         return false, run once
         */
        void addTask(const std::function<bool()>& func,float delay_ms = 100){
            task_queue.emplace_back(func,delay_ms);

        }
        bool call(){
            common::Time now = common::FromUnixNow();

            bool run = false;

            if (!task_queue.empty()) {

                for(auto& task_opt: task_queue)
                {
                    if(common::ToMillSeconds(now - task_opt.time) >= task_opt.delay_ms){
                        run = true;
//                        std::cout << "run TaskManager time " << common::getCurrentDateTime();
                        task_opt.valid = task_opt.func();
                        if(task_opt.valid){
                            task_opt.time = now;
                        }
                    }

                }
                if(run){

                    auto it  = std::remove_if(task_queue.begin(),task_queue.end(),[](auto& e){
                        return !e.valid;
                    });

                    task_queue.erase(it, task_queue.end());
                }


            }

            return !task_queue.empty();

        }

    };
}
#endif //SCAN_REPUBLISHER_TASK_H
