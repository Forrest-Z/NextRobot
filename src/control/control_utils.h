//
// Created by waxz on 23-2-15.
//

#ifndef SCAN_REPUBLISHER_CONTROL_UTILS_H
#define SCAN_REPUBLISHER_CONTROL_UTILS_H

#include <vector>
#include "transform/transform.h"


namespace control {

    transform::Transform2d updateCurveDist(const transform::Transform2d& init_pose, float curve, float dist);

    template<typename FloatType>
    transform::MatrixSE2<FloatType> updateCurveDistDiff(const transform::MatrixSE2<FloatType>& init_pose, FloatType curve, FloatType dist){
        transform::MatrixSE2<FloatType> result_pose;

        if(abs(curve) < 1e-12){
            result_pose.set(dist, 0.0,0.0);
            result_pose = init_pose *result_pose;
        }else{
            FloatType radius = 1.0/(curve);
            FloatType angle = dist* curve;

            transform::MatrixSE2<FloatType> rotate_center_relative(0.0, radius, 0.0);
            transform::MatrixSE2<FloatType> rotate_center_absolute = init_pose*rotate_center_relative;
            FloatType angle_relative = (curve>0.0) ? (angle - M_PI_2) : ( M_PI_2 + angle);

            FloatType abs_radius = abs(radius);
            transform::MatrixSE2<FloatType> result_pose_relative(abs_radius*cos(angle_relative), abs_radius*sin(angle_relative), angle);

            result_pose = rotate_center_absolute * result_pose_relative;
        }
        return result_pose;
    }


    struct ControlPath {
        std::vector<transform::Transform2d> path;
        float curve = 0.0;
        float dist = 0.0;
        size_t current_id = 0;

    };

    
    
    void interpolatePath(const transform::Transform2d& init_pose, float curve, float dist, int step_num, std::vector<transform::Transform2d>& path);


}

#endif //SCAN_REPUBLISHER_CONTROL_UTILS_H
