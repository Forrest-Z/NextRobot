//
// Created by waxz on 23-2-8.
//

#ifndef SCAN_REPUBLISHER_OPTIM_CIRCLE_SOLVER_H
#define SCAN_REPUBLISHER_OPTIM_CIRCLE_SOLVER_H

#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "sensor/geometry_types.h"

namespace perception{
    void FindCircle(std::vector<geometry::Point>& points,float radius, float edge_radius, float edge_range_offset, int min_point_num, float & center_x, float & center_y, float& error_mean);

}

#endif //SCAN_REPUBLISHER_OPTIM_CIRCLE_SOLVER_H
