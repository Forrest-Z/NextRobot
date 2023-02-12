//
// Created by waxz on 23-2-4.
//

#ifndef SCAN_REPUBLISHER_LASERSCAN_FILTER_H
#define SCAN_REPUBLISHER_LASERSCAN_FILTER_H

#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "sensor/geometry_types.h"
namespace perception{
    class RangesFilter{

    public:
        void add(const std::vector<float>& ranges){
            int point_num = ranges.size();
            if(!is_init){
                matrix_buffer = Eigen::MatrixXf(max_len,point_num);
//            ranges_buffer.resize(max_len, ranges);
                is_init = true;
            }
            if(update_index >=max_len){
                return;
            }
            for(int i = 0 ; i < point_num;i++){
                matrix_buffer(update_index,i) = ranges[i];
            }
//        ranges_buffer[update_index] = ranges;

            update_index++;

        }
        bool filter(){
            if(update_index < max_len){
                return false;
            }
            Eigen::VectorXf mean_vector = matrix_buffer.colwise().mean();

            Eigen::MatrixXf centered = matrix_buffer.rowwise() - mean_vector.transpose();
//        Eigen::Array<double, 1, Eigen::Dynamic> std_dev = (centered.square().colwise().sum()/(M-1)).sqrt();
            auto std_dev = centered.array().pow(2).colwise().sum() /(max_len -1);
            int point_num = std_dev.cols();
            ranges_filtered.resize(point_num);
            for(int i = 0 ; i <point_num;i++){
                ranges_filtered[i] = std_dev(0,i) < max_stddev ? mean_vector(i):100.0;
            }
            return true;

        }
        const std::vector<float>& getFiltered(){
            return ranges_filtered;
        }
        void clear(){
            update_index = 0;
        }

    public:
        int max_len = 10;
        float max_stddev = 0.01;
        int update_index = 0;

    private:
        bool is_init = false;
        std::vector<std::vector<float>> ranges_buffer;
        std::vector<float> ranges_filtered;
        std::vector<float> ranges_stddev;

        Eigen::MatrixXf matrix_buffer;

    };


    /*
 box filter
 remove points outside of box
 */


    void LaserScanBoxFilter(const std::vector<float> &scan_points, int point_num,
                            std::vector<geometry::Point>& filter_points, int& filter_point_num,
                            float min_x, float max_x, float min_y, float max_y
    );
    void LaserScanBoxFilter(const std::vector<float> &scan_points_laser, const std::vector<float> &scan_points_base, int point_num,
                            std::vector<geometry::Point>& filter_points_laser, std::vector<geometry::Point>& filter_points_base, int& filter_point_num,
                            float min_x, float max_x, float min_y, float max_y
    );


    void LaserScanRadiusFilter(const std::vector<float> &scan_points_laser, const std::vector<float> &scan_points_base, int point_num,
                                      std::vector<geometry::Point>& filter_points_laser, std::vector<geometry::Point>& filter_points_base, int& filter_point_num,
                                      float relative_x, float relative_y, float filter_radius
    );

    void LaserScanSegment( const std::vector<geometry::Point>& filter_points, std::vector<std::vector<geometry::Point>>& segments, int filter_point_num, float split_dist_x,float split_dist_y,float split_dist_r, int min_point_in_seg_num,
                           float shadow_angle  = 1.3
    );

}
#endif //SCAN_REPUBLISHER_LASERSCAN_FILTER_H
