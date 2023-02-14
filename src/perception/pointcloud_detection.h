//
// Created by waxz on 23-1-29.
//

#ifndef SCAN_REPUBLISHER_POINTCLOUD_DETECTION_H
#define SCAN_REPUBLISHER_POINTCLOUD_DETECTION_H

#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include "sensor/geometry_types.h"


namespace perception{

    // find circle in point cluster



    /*
     find front egge in pont cluster
     point should be in laser frame
     1. first find the closest point
     2. filter points in radius
     3. compute center
     4. filter points in radius
     5. compute distribution, remove outlier points
     6. repeat 2-5 ,util finally no outlier found
     */

    void FindFrontEdge(std::vector<geometry::Point>& points,int max_iter_num,float radius, int min_point_num, float & center_x, float & center_y, float& error_mean);


    /*

     find pattern match given model
     model_data: shelf_leg or free_installed_light_marker, represent model as vector<Point>
     match_num define minimum matched pair, so model_data should be sorted so the import point locate at the front of the vector
     input data represent as vector<vector<Point>>

     if import points match fail, then whole process match fail



     match_result : [
     [[0,-1,2], 2, 0.05,0.06],

     []
     ]

     find best match result
     */



    bool FindMatchPattern(const std::vector<geometry::DetectTarget>& model_points, const std::vector<std::vector<geometry::DetectTarget>>& input_points,float match_radius , int min_matched_num , std::vector<std::array<int,10>>& match_result  );

    // fine line in point cluster







}

#endif //SCAN_REPUBLISHER_POINTCLOUD_DETECTION_H
