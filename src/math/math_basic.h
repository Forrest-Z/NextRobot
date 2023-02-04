//
// Created by waxz on 23-1-31.
//

#ifndef SCAN_REPUBLISHER_MATH_BASIC_H
#define SCAN_REPUBLISHER_MATH_BASIC_H

#include <cmath>
#define angle_normalise_zero(angle) std::abs(angle) < M_PI ? (angle) : ( angle + ((angle) > 0.0 ? -M_PI*2: M_PI*2) )
#define angle_normalise(angle, angle_mean)  ((std::abs(angle - angle_mean ) < M_PI )? (angle) : ( angle + ((angle - angle_mean) > 0.0 ? -M_PI*2.0: M_PI*2.0) ))

namespace math{


    inline void to_yaw( float x,  float y,  float z,  float w, float &yaw) {
// roll (x-axis rotation)
        float sinr_cosp = +2.0 * (w * x + y * z);
        float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);

// pitch (y-axis rotation)
        float sinp = +2.0 * (w * y - z * x);


// yaw (z-axis rotation)
        float siny_cosp = +2.0 * (w * z + x * y);
        float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
    }
    inline void toEulerAngle(const float x, const float y, const float z, const float w, float &roll, float &pitch, float &yaw) {
// roll (x-axis rotation)
        float sinr_cosp = +2.0 * (w * x + y * z);
        float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
        float sinp = +2.0 * (w * y - z * x);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

// yaw (z-axis rotation)
        float siny_cosp = +2.0 * (w * z + x * y);
        float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
    }


    inline void yaw_to_quaternion(float yaw, double &qx, double &qy, double &qz, double &qw) {
        float roll = 0.0;
        float pitch = 0.0;

        qx = 0.0;
        qy = 0.0;
        qz = sin(yaw * 0.5f);
        qw = cos(yaw * 0.5f);

        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

    }
    inline void yaw_to_quaternion(float yaw, float &qx, float &qy, float &qz, float &qw) {
        float roll = 0.0;
        float pitch = 0.0;

        qx = 0.0;
        qy = 0.0;
        qz = sin(yaw * 0.5f);
        qw = cos(yaw * 0.5f);

        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

    }
}

#endif //SCAN_REPUBLISHER_MATH_BASIC_H
