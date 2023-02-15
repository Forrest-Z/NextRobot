//
// Created by waxz on 23-2-15.
//

#ifndef SCAN_REPUBLISHER_OPTIM_PATH_PLANNER_H
#define SCAN_REPUBLISHER_OPTIM_PATH_PLANNER_H

#include <Eigen/Dense>
#include "transform/transform.h"
#include "control_utils.h"


namespace control{

    struct PathPlanner{
        float x_tolerance = 0.02;
        float y_tolerance = 0.02;
        float yaw_tolerance = 0.02;
        int curve_num =3;
        std::vector<ControlPath> path_segments;
        ControlPath local_path_segment;

        Eigen::VectorXd solved_params;
        bool params_compute_ok = false;
        bool local_params_compute_ok = false;


        bool solve(const transform::Transform2d& robot_init_pose, const transform::Transform2d& target_pose);
        bool computeLocalCurve(const transform::Transform2d& robot_init_pose, float& curve, float& dist);
    };




}

#endif //SCAN_REPUBLISHER_OPTIM_PATH_PLANNER_H
