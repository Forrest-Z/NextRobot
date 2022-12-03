//
// Created by waxz on 22-10-12.
//

#ifndef SCAN_REPUBLISHER_TYPE_CAST_H
#define SCAN_REPUBLISHER_TYPE_CAST_H

#include <type_traits>
#include <vector>
#include <array>

namespace common{


    template<typename T1, typename T2,
            typename std::enable_if<std::is_constructible<T1,T2>{},
bool>::type = true>
void simple_cast( T1& value1, const T2 &value2    ){

    value1 = static_cast<T1>(value2);
}

template<typename T1, typename T2, long unsigned int N>
void simple_cast( std::array<T1,N> & value1, const std::array<T2,N> &value2    ){
    for(int i = 0; i < value2.size();i++){
        simple_cast(value1[i],value2[i]);
    }
}


template<typename T1, typename T2>
void simple_cast( std::vector<T1> & value1, const std::vector<T2> &value2    ){
    value1.resize(value2.size());
    for(int i = 0; i < value2.size();i++){
        simple_cast(value1[i],value2[i]);
    }
}


}
#endif //SCAN_REPUBLISHER_TYPE_CAST_H
