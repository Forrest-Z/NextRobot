//
// Created by waxz on 23-2-15.
//

#ifndef SCAN_REPUBLISHER_CONTROL_UTILS_H
#define SCAN_REPUBLISHER_CONTROL_UTILS_H

#include <vector>
#include "transform/transform.h"


namespace control {

    transform::Transform2d updateCurveDist(const transform::Transform2d& init_pose, float curve, float dist){
        transform::Transform2d result_pose;

        //FLT_EPSILON =  1.192093e-07
        // 1/FLT_EPSILON  is beyond float limit
        if(std::abs(curve) < 1e-5){
            result_pose.set(dist, 0.0,0.0);
            result_pose = init_pose *result_pose;
        }else{
            float radius = 1.0f/curve;
            float angle = dist* curve;

            transform::Transform2d rotate_center_relative(0.0, radius, 0.0);
            transform::Transform2d rotate_center_absolute = init_pose*rotate_center_relative;
            float angle_relative = (curve>0.0) ? (angle - M_PI_2) : ( M_PI_2 + angle);

            float abs_radius = std::abs(radius);
            transform::Transform2d result_pose_relative(abs_radius*std::cos(angle_relative), abs_radius*std::sin(angle_relative), angle);

            result_pose = rotate_center_absolute * result_pose_relative;
        }
        return result_pose;
    }

    void interpolatePath(const transform::Transform2d& init_pose, float curve, float dist, int step_num, std::vector<transform::Transform2d>& path){
        path.resize(step_num);
        float dist_inc = dist/static_cast<float>(step_num);
        for(int i = 0 ; i < step_num -1; i++){
            path[i]= updateCurveDist(init_pose, curve, static_cast<float>(i+1) * dist_inc);
        }
        path[step_num -1 ]= updateCurveDist(init_pose, curve,dist);
    }


}

#endif //SCAN_REPUBLISHER_CONTROL_UTILS_H
