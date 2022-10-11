//
// Created by waxz on 22-10-11.
//

#ifndef SCAN_REPUBLISHER_GENERATOR_H
#define SCAN_REPUBLISHER_GENERATOR_H

#include <vector>
#include <algorithm>

namespace FP{
    template <typename T>
    void lin_space(T start, T inc, int n, std::vector<T>& data){
        data.resize(n);
        std::for_each(data.begin(),data.end(),[&, inc = inc,start = start](auto& e)mutable {
            e = start;
            start += inc;
        });
    }
}

#endif //SCAN_REPUBLISHER_GENERATOR_H
