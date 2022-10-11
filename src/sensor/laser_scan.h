//
// Created by waxz on 22-10-11.
//

#ifndef SCAN_REPUBLISHER_LASER_SCAN_H
#define SCAN_REPUBLISHER_LASER_SCAN_H

#include <vector>
#include <array>
#include <cmath>
#include "../transform/transform.h"
#include "../functional/generator.h"
namespace sensor{
    struct ScanToPoints{

        std::vector<float> angle_buffer;
        std::vector<float> cos_angle_buffer;
        std::vector<float> sin_angle_buffer;
        std::vector<float> local_xy_points;
        std::vector<float> global_xy_points;
        std::vector<std::array<float,3>> global_xy_points_vec;
        transform::Transform2d transform2D;
        std::vector<float> ranges;
        std::vector<float> intensities;

        std::vector<float> ranges_filtered;
        std::vector<std::array<int,2>> filter_cluster_range;
        std::vector<std::vector<int>> filter_cluster_index;


        float scan_range_max;
        float scan_angle_inc_inv;


        struct FilterOption{
            float radius = 0.4;
            float min_intensity = 600.0;
            int min_light_num = 5;
            int max_light_num = 20;
        };

        FilterOption filterOption;



        bool init_done = false;
        void getLocalPoints(  const  std::vector<float>& scan_ranges, const   std::vector<float>& scan_intensities, float angle_min, float  angle_increment, float range_max){

            int N = scan_ranges.size();

            ranges =  std::remove_const_t<std::vector<float>>(scan_ranges)  ;
//            intensities = std::remove_const_t<std::vector<float>>(scan_intensities)  ;

            if(init_done){


                for(int i = 0 ; i < N;i++){
                    ranges[i] = std::isnormal(ranges[i]) ? ranges[i]:range_max ;
                }

                for(int i = 0 ; i < N;i++){
                    local_xy_points[i+i] = ranges[i]*cos_angle_buffer[i];
                    local_xy_points[i+i +1] = ranges[i]*sin_angle_buffer[1];

                }
#if 1
                if(scan_intensities.size() == N){
                    for(int i = 0 ; i < N;i++){
                        global_xy_points_vec[i][2] =  scan_intensities[i]  ;
                    }
                }
#endif


            }else{


                FP::lin_space(angle_min,angle_increment,N,angle_buffer);
                cos_angle_buffer.resize(N);
                sin_angle_buffer.resize(N);
                local_xy_points.resize(N+N);
                global_xy_points.resize(N+N);
                global_xy_points_vec.resize(N,std::array<float,3>({0.0,0.0,0.0}));

                for(int i = 0 ; i < N;i++){
                    cos_angle_buffer[i] = cos(angle_buffer[i]);

                }
                for(int i = 0 ; i < N;i++){
                    sin_angle_buffer[i] = sin(angle_buffer[i]);

                }

                scan_range_max = range_max;
                scan_angle_inc_inv = 1.0/angle_increment;
                ranges_filtered.resize(N,scan_range_max); //scan_range_max

                init_done = true;

            }

        }
        void getGlobalPoints(float x, float y, float yaw){
            transform2D.set(x,y,yaw);
            transform2D.mul(local_xy_points,global_xy_points);

            int N= global_xy_points_vec.size();


            for(int i = 0 ; i < N ;i++){
                global_xy_points_vec[i][0] = global_xy_points[i+i];
                global_xy_points_vec[i][1] = global_xy_points[i+i+1];

            }

        }

        void getGlobalPoints(){

            int N= global_xy_points_vec.size();

            for(int i = 0 ; i < N ;i++){
                global_xy_points_vec[i][0] = local_xy_points[i+i];
                global_xy_points_vec[i][1] = local_xy_points[i+i+1];
            }

        }

        void filterMarker(const std::vector<std::array<float,3>> & markers){


            // markers: markers position in laser frame
            // [x1,y1,i1],[x2,y2,i2]......


            // filter option
            // radius
            // min_intensity
            // min_light_num
            // max_light_num
            // kmeans param

            int N= markers.size();

            std::cout << "start filter\n";
            std::cout << "N : " << N <<std::endl;
            std::cout << "ranges.size() : " << ranges.size() <<std::endl;
            std::cout << "ranges_filtered.size() : " << ranges_filtered.size() <<std::endl;

            filter_cluster_range.clear();
            filter_cluster_index.resize(N);

            for(int i = 0; i < N;i++){
                filter_cluster_index[i].clear();


                float m_angle = atan2(markers[i][1],markers[i][0]);
                float m_range = sqrt(markers[i][0]*markers[i][0] + markers[i][1]*markers[i][1]);
                float angle_offset = asin(filterOption.radius/m_range);

                auto lb = std::lower_bound(angle_buffer.begin(),angle_buffer.end(),m_angle-angle_offset);
                auto ub = std::upper_bound(angle_buffer.begin(),angle_buffer.end(),m_angle+angle_offset);

                int lb_index = (m_angle-angle_offset - angle_buffer[0])*scan_angle_inc_inv;
                int ub_index = (m_angle+angle_offset - angle_buffer[0])*scan_angle_inc_inv;
                ub_index = ub_index>angle_buffer.size()-1 ? angle_buffer.size()-1:ub_index;


                std::cout << "m_angle: " << m_angle << ", m_range: " << m_range << ", angle_offset: " << angle_offset << std::endl;
                std::cout << "lb: " << std::distance(angle_buffer.begin(), lb) << "ub: " << std::distance(angle_buffer.begin(), ub) << std::endl;
                std::cout << "lb_index: " << lb_index  << "ub_index: " << ub_index<< std::endl;

                if(lb_index>=0){
                    for(int j =lb_index; j < ub_index;j++){
                        if(  (fabs(ranges[j] - m_range) < filterOption.radius) &&( global_xy_points_vec[j][2] > filterOption.min_intensity   ) ){
                            filter_cluster_index[i].push_back(j);
                        }
                    }
                }else{
                    for(int j =0; j < ub_index;j++){
                        if( ( fabs(ranges[j] - m_range) < filterOption.radius)&&( global_xy_points_vec[j][2] > filterOption.min_intensity   ) ){
                            filter_cluster_index[i].push_back(j);
                        }
                    }
                    for(int j =ranges.size() + lb_index; j < ranges.size();j++){
                        if( (fabs(ranges[j] - m_range) < filterOption.radius )&&( global_xy_points_vec[j][2] > filterOption.min_intensity   )){
                            filter_cluster_index[i].push_back(j);
                        }
                    }

                }


                filter_cluster_range.emplace_back(std::array<int,2>{lb_index,ub_index});

            }


//            ranges_filtered.resize(ranges.size(), scan_range_max);
            std::fill(ranges_filtered.begin(), ranges_filtered.end(),scan_range_max);


#if 0
            for(int i = 0; i < N;i++){

//                std::copy(ranges.begin() +filter_cluster_range[i][0], ranges.begin() +filter_cluster_range[i][1],ranges_filtered.begin() +filter_cluster_range[i][0] );
//                std::copy(ranges.begin() +10, ranges.begin() +1000,ranges_filtered.begin() +10 );
//                std::copy(ranges.begin() +filter_cluster_range[i][0], ranges.begin() +filter_cluster_range[i][0] + 100,ranges_filtered.begin() +filter_cluster_range[i][0] );
//                std::copy(ranges.begin() , ranges.begin() +filter_cluster_range[i][1],ranges_filtered.begin()  );

                if(filter_cluster_range[i][0] >=0){
                    std::copy(ranges.begin()+filter_cluster_range[i][0], ranges.begin()+filter_cluster_range[i][1],ranges_filtered.begin()+filter_cluster_range[i][0]);
                }else{
                    std::copy(ranges.begin(), ranges.begin()+filter_cluster_range[i][1],ranges_filtered.begin());
                    std::copy(ranges.begin()+filter_cluster_range[i][0] + ranges.size(), ranges.end(),ranges_filtered.begin()+filter_cluster_range[i][0]+ ranges.size());

                }

            }
#endif

            for(int i = 0; i < N;i++){
                for(auto& j :filter_cluster_index[i]){
                    ranges_filtered[j] = ranges[j];
                }

            }


//            ranges_filtered = ranges;
        }


    };
}

#endif //SCAN_REPUBLISHER_LASER_SCAN_H
