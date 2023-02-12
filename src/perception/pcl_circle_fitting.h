//
// Created by waxz on 23-2-12.
//

#ifndef SCAN_REPUBLISHER_PCL_CIRCLE_FITTING_H
#define SCAN_REPUBLISHER_PCL_CIRCLE_FITTING_H


#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "sensor/geometry_types.h"

namespace perception{
    void FindCirclePcl(std::vector<geometry::Point>& points,float radius, float edge_radius, float edge_range_offset, int min_point_num, float & center_x, float & center_y, float& error_mean);

}

#endif //SCAN_REPUBLISHER_PCL_CIRCLE_FITTING_H
