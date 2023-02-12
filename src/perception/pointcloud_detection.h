//
// Created by waxz on 23-1-29.
//

#ifndef SCAN_REPUBLISHER_POINTCLOUD_DETECTION_H
#define SCAN_REPUBLISHER_POINTCLOUD_DETECTION_H

#include <vector>

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

    void FindFrontEdge(std::vector<geometry::Point>& points,float radius, int min_point_num, float & center_x, float & center_y, float& error_mean){

        center_x = 100000.0;
        center_y = 100000.0;
        if(points.size() < min_point_num){
            std::cout << "check points.size() fail return : " << points.size()  << std::endl;

            error_mean = 100.0;
            return;
        }
        std::sort(points.begin(),points.end(), [](auto& v1, auto& v2){ return v1.r < v2.r; });



        float cx = points[1].x;
        float cy = points[1].y;
        float sum_x = 0.0, sum_y = 0.0;
        int valid_num = 0;

        float radius_2 = radius *radius;

        for(auto&p : points){
            float d = ( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
            if(d < radius_2){
                sum_x += p.x;
                sum_y += p.y;
                valid_num++;
            }
        }
        if(valid_num < min_point_num){
            std::cout << "check valid_num fail return" << std::endl;

            error_mean = 100.0;
            return;
        }

        cx = sum_x/float(valid_num);
        cy = sum_y/float(valid_num);
        valid_num = 0;
        sum_x = 0.0;
        sum_y = 0.0;

        for(auto&p : points){
            float d = ( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
            if(d < radius_2){
                sum_x += p.x;
                sum_y += p.y;
                valid_num++;
            }
        }
        cx = sum_x/float(valid_num);
        cy = sum_y/float(valid_num);

        center_x = cx;
        center_y = cy;
        error_mean = 0.0;
    }


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



    bool FindMatchPattern(const std::vector<geometry::DetectTarget>& model_points, const std::vector<std::vector<geometry::DetectTarget>>& input_points,float match_radius , int min_matched_num , std::vector<std::array<int,10>>& match_result  ){


        // check important marker
        // if no input cluster
        // return fail


        // check min_matched_num
        int exist_num = 0;
        for(size_t i = 0 ; i < input_points.size(); i++){
            exist_num += ! input_points.empty();
        }
        if(exist_num < min_matched_num){
            return false;
        }

//        match_result.resize(model_points.size());
        std::tuple<std::vector<int>, int, float,float  > one_match;
//        match_result.fill(match_result.begin(),match_result.end(),  one_match);

        int m_i = 0, m_j = 0, m_k = 0;
        float dist1 = 0.0, dist2 = 0.0, diff = 0.0;
        float match_radius_2 = match_radius*match_radius;


        std::cout << __FUNCTION__ << "check important index , match_radius = " << match_radius << "\n";
        match_result.clear();

        // mode_index1, model_index2, cluster_index1, cluster_index2, diff*1000
        std::vector<std::vector< std::vector<std::array<int,3>> >> match_edges;
        match_edges.resize(model_points.size(), std::vector<std::vector<std::array<int,3>> >(model_points.size()));
        std::array<int, 3> match_index;


        std::vector<std::set<int>> model_query_index(model_points.size());

        for(size_t m_i = 0 ; m_i < model_points.size();m_i++){

            if(input_points[m_i].empty()){
                continue;
            }

            for(size_t m_j = m_i+1 ; m_j < model_points.size();m_j++){
                if(input_points[m_j].empty()){
                    continue;
                }

                std::cout << "[m_i, m_j] = [" << m_i << ", " << m_j << "]\n";


                std::cout << "[input_points[m_i].size(), input_points[m_j].size()] = [" << input_points[m_i].size() << ", " << input_points[m_j].size() << "]\n";


                dist1 =  std::sqrt( (model_points[m_i].pose_in_base[0] -model_points[m_j].pose_in_base[0] )*(model_points[m_i].pose_in_base[0] -model_points[m_j].pose_in_base[0] )
                    + (model_points[m_i].pose_in_base[1] -model_points[m_j].pose_in_base[1] )*(model_points[m_i].pose_in_base[1] -model_points[m_j].pose_in_base[1] ));



                for(size_t k = 0 ; k < input_points[m_i].size(); k++){

                    for(size_t l = 0 ; l < input_points[m_j].size(); l++){


                        std::cout << "[k, l] = [" << k << ", " << l << "]\n";

                        dist2 =  std::sqrt(
                                (input_points[m_i][k].pose_in_base[0] -input_points[m_j][l].pose_in_base[0] )*(input_points[m_i][k].pose_in_base[0] -input_points[m_j][l].pose_in_base[0] )
                                + (input_points[m_i][k].pose_in_base[1] -input_points[m_j][l].pose_in_base[1] )*(input_points[m_i][k].pose_in_base[1] -input_points[m_j][l].pose_in_base[1] )
                        );


                        std::cout << "[dist1, dist2] = [" << dist1 << ", " << dist2 << "]\n";

                        float diff = std::abs(dist1 - dist2);

                        if(diff < match_radius){
                            // match one edge
                            match_index[0] = k;
                            match_index[1] = l;
                            match_index[2] = 1000*diff;
                            match_edges[m_i][m_j].emplace_back(match_index);

                            model_query_index[m_i].insert(k);
                            model_query_index[m_j].insert(l);

                        }
                    }


                }


            }
        }


        std::cout << "check match_edges:\n";
        for(size_t i = 0 ; i < match_edges.size();i++){
            for(size_t j = 0 ; j < match_edges[i].size();j++){
                for(auto& e : match_edges[i][j]){
                    std::cout << "* " << i << ", " << j << ", " << e[0] << ", " << e[1] << ", " << e[2] << "\n";
                }
            }
        }


        std::cout << "check match_edges done" << std::endl;


        std::array<int,10> select_points_index;
        select_points_index.fill(-1);
        std::vector<std::array<int,10>> select_points_index_group;


        size_t all_possible_num = 1;

        size_t last_branch_num = 1;

        size_t current_branch_num = 1;

        size_t array_write_head_start = 0, array_write_head_end = 0;
        size_t array_write_num = 1;
        size_t node_num = 1;

        for(size_t i = 0 ; i < model_query_index.size();i++){
            all_possible_num *= (model_query_index[i].size() > 0) ? model_query_index[i].size() : 1;
        }

        select_points_index_group.resize(all_possible_num,select_points_index);


        for(size_t i = 0 ; i < model_query_index.size();i++){

            node_num = model_query_index[i].size();
            current_branch_num *= ( node_num > 0) ? model_query_index[i].size() : 1;
            array_write_num = 0;

            for(auto& p:model_query_index[i]){

                std::cout << "* " << i << ", "<< p << "\n";
                array_write_head_start = array_write_num*last_branch_num;
                array_write_head_end = array_write_head_start + last_branch_num;

// repeat
                if(array_write_num > 0){

                    for(size_t j = array_write_head_start; j < array_write_head_end;j++){

                        select_points_index_group[j] = select_points_index_group[j-array_write_head_start];

                    }
                }
// update

                for(size_t j = array_write_head_start; j < array_write_head_end;j++){
                    select_points_index_group[j][i] = p;
                }

                array_write_num ++ ;
            }
            last_branch_num = current_branch_num;
        }


        std::cout << "check all_possible_num  " << all_possible_num << std::endl;



        std::vector<std::array<int,10>> final_select_points_index_group;

        std::cout << "check select_points_index_group: " << select_points_index_group.size() << "\n";


        for(auto &row : select_points_index_group){


            bool match_fail = false;
            float match_error = 0.0;
            int match_num = 0;

            for( m_i = 0 ; m_i < model_points.size();m_i++){

                size_t k = row[m_i];

                if(k == -1){
                    continue;
                }
                for( m_j = m_i+1 ; m_j < model_points.size();m_j++){

                    size_t l = row[m_j];
                    if(l  == -1){
                        continue;
                    }



                    dist1 =  std::sqrt( (model_points[m_i].pose_in_base[0] -model_points[m_j].pose_in_base[0] )*(model_points[m_i].pose_in_base[0] -model_points[m_j].pose_in_base[0] )
                            + (model_points[m_i].pose_in_base[1] -model_points[m_j].pose_in_base[1] )*(model_points[m_i].pose_in_base[1] -model_points[m_j].pose_in_base[1] ));


                    dist2 =  std::sqrt( (input_points[m_i][k].pose_in_base[0] -input_points[m_j][l].pose_in_base[0] )*(input_points[m_i][k].pose_in_base[0] -input_points[m_j][l].pose_in_base[0] )
                            + (input_points[m_i][k].pose_in_base[1] -input_points[m_j][l].pose_in_base[1] )*(input_points[m_i][k].pose_in_base[1] -input_points[m_j][l].pose_in_base[1] ));



                    std::cout << "[dist1, dist2] = [" << dist1 << ", " << dist2 << "]\n";

                    float diff = std::abs(dist1 - dist2);
                    match_error += diff;


                    match_fail = diff > match_radius;
                    if(match_fail)break;


                }

                if(match_fail)break;





            }

            std::cout << "match_fail " << match_fail << "\n";

            if(!match_fail){


                exist_num = 0;
                for(size_t i = 0 ; i < input_points.size(); i++){
                    exist_num += row[i]!= -1;
                }
                if(exist_num >=  min_matched_num){
                    row.back() = 1000*match_error/model_points.size() ;
                    match_result.emplace_back(row);
                }



//                    final_select_points_index_group.emplace_back(row);

            }

            std::cout << "---\n";
            for(auto& n : row){
                std::cout << n << ", ";
            }

            std::cout << "---\n";

        }


        std::cout << "check model_query_index done " << std::endl;


        return true;

    }

    // fine line in point cluster







}

#endif //SCAN_REPUBLISHER_POINTCLOUD_DETECTION_H
