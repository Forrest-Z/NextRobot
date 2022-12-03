//
// Created by waxz on 9/24/22.
//

#ifndef CMAKE_SUPER_BUILD_SERIALIZATION_JSON_H
#define CMAKE_SUPER_BUILD_SERIALIZATION_JSON_H
#include <tuple>
#include <type_traits>
#include "nlohmann/json.hpp"


namespace serialization{

    template<typename Class, typename T>
    struct PropertyImpl {
        constexpr PropertyImpl(T Class::*aMember, const char* aName) : member{aMember}, name{aName} {}

        using Type = T;

        T Class::*member;
        const char* name;
    };

    template<typename Class, typename T>
    constexpr auto property(T Class::*member, const char* name) {
        return PropertyImpl<Class, T>{member, name};
    }

    template <typename T, T... S, typename F>
    constexpr void for_sequence(std::integer_sequence<T, S...>, F&& f) {
        using unpack_t = int[];
        (void)unpack_t{(static_cast<void>(f(std::integral_constant<T, S>{})), 0)..., 0};
    }


    template<typename T,
            typename std::enable_if<std::is_constructible<nlohmann::json,T>{},
                    bool>::type = true
    >
    void to_json(nlohmann::json& data,const std::string& name , const T& object ){
        data[name] = object;
    }

    template<typename T,
            typename std::enable_if<std::is_constructible<nlohmann::json,T>{},
                    bool>::type = true
    >
    bool from_json(nlohmann::json& data,const std::string& name ,  T& object ){
        if(data.contains(name)){
            object = data[name].template get<T>();
            return true;
        }else{
            return false;
        }
        return true;

    }


    template<typename T,typename PROP>
    bool to_json_unpack(nlohmann::json& data,const std::string& name , const T & object , const PROP & properties){

        constexpr auto nbProperties = std::tuple_size<PROP>::value;

        for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(properties);
            // set the value to the member
            to_json(data[name],property.name, object.*(property.member));
        });
        return true;
    }
    template<typename T,typename PROP>
    bool from_json_unpack(nlohmann::json& data,const std::string& name ,  T& object , const PROP & properties){
        constexpr auto nbProperties = std::tuple_size<PROP>::value;
        // We iterate on the index sequence of size `nbProperties`
        bool rt = true;
        for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(properties);
            // you can also replace `asAny` by `fromJson` to recursively serialize
            using Type = typename std::remove_reference<decltype(object.*(property.member))>::type;
            rt = rt && from_json(data[name],property.name,object.*(property.member));
        });
        return rt;

    }
}


#endif //CMAKE_SUPER_BUILD_SERIALIZATION_JSON_H
