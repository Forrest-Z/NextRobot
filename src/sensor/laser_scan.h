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
        std::vector<float> intensities_vec;
        std::vector<float> global_xy_points;
        std::vector<std::array<float,3>> global_xy_points_vec;
        transform::Transform2d transform2D;
        std::vector<float> ranges;
        std::vector<float> intensities;

        std::vector<float> ranges_filtered;
        std::vector<std::array<int,2>> filter_cluster_range;
        std::vector<std::vector<int>> filter_cluster_index;

        std::vector<std::vector<std::array<float,3>>> filter_cluster_points;

        std::vector<std::array<float,3>>  filter_markers;

        std::vector<int> ranges_valid_index;
        int range_valid_num = 0;


        float scan_range_max;
        float scan_angle_inc_inv;
        float scan_max_jump = 0.06;
        float arc_inc = 0.01;
        float scan_noise_angle = 0.05;


        struct FilterOption{
            float radius = 0.4;
            float min_intensity = 600.0;
            int min_light_num = 5;
            int max_light_num = 20;
            float marker_radius = 0.05;
            float edge_offset = 0.1;
            int min_edge_num = 2;

        };

        FilterOption filterOption;



        bool init_done = false;
        void getLocalPoints(  const  std::vector<float>& scan_ranges, const   std::vector<float>& scan_intensities, float angle_min, float  angle_increment, float range_max){

            int N = scan_ranges.size();

            ranges =  std::remove_const_t<std::vector<float>>(scan_ranges)  ;
//            intensities = std::remove_const_t<std::vector<float>>(scan_intensities)  ;


            if (!init_done){


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
                arc_inc = sin(angle_increment);
                ranges_filtered.resize(N,scan_range_max); //scan_range_max

                init_done = true;

            }

            {


                for(int i = 0 ; i < N;i++){
                    ranges[i] = std::isnormal(ranges[i]) ? ranges[i]:range_max ;
                }

                for(int i = 0 ; i < N;i++){
                    local_xy_points[i+i] = ranges[i]*cos_angle_buffer[i];
                    local_xy_points[i+i +1] = ranges[i]*sin_angle_buffer[i];

                }
#if 1
                if(scan_intensities.size() == N){
                    for(int i = 0 ; i < N;i++){
                        global_xy_points_vec[i][2] =  scan_intensities[i]  ;
                    }
                }
#endif


            }

        }

        void getLocalPoints(  const  std::vector<float>& scan_ranges, const   std::vector<float>& scan_intensities, float angle_min, float  angle_increment, float range_min, float range_max){

            int N = scan_ranges.size();

//            ranges =  std::remove_const_t<std::vector<float>>(scan_ranges)  ;
            auto & ranges_ =   scan_ranges  ;

//            intensities = std::remove_const_t<std::vector<float>>(scan_intensities)  ;


            if (!init_done){


                FP::lin_space(angle_min,angle_increment,N,angle_buffer);
                cos_angle_buffer.resize(N);
                sin_angle_buffer.resize(N);
                intensities_vec.resize(N);
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
                arc_inc = sin(angle_increment);

                ranges_filtered.resize(N,scan_range_max); //scan_range_max
                ranges_valid_index.resize(N);
                init_done = true;

            }



            {

                range_valid_num = 0;

                bool valid = false;
                int offset = 3;
//                std::cout << "\n***********check range_valid_num\n";
                for(int i = offset ; i < N-offset;i++){
                    ranges_valid_index[range_valid_num] = i;
//                    std::cout << ranges_[i] << ", ";
                    valid = std::isnormal(ranges_[i])  &&(ranges_[i] < range_max) && (ranges_[i] > range_min)
                            && (  (std::abs(ranges_[i]-ranges_[i-1]) < scan_max_jump)|| (std::abs(ranges_[i]-ranges_[i+1]) < scan_max_jump) )
                            && ( std::abs( ranges_[i+1] - ranges_[i-1] ) < scan_max_jump)
                            && ( std::atan2( offset*arc_inc * ranges_[i],std::abs(ranges_[i+1] - ranges_[i-1])    ) > scan_noise_angle )
                            ;

                    range_valid_num += valid;

                }
//                range_valid_num++;

//                std::cout << "\n***********check range_valid_num : " << range_valid_num << std::endl;

#if 1
                for(int i = 1 ; i < range_valid_num-1;i++){
                    int j = ranges_valid_index[i];
                    float r = ranges_[j];
                    float d1 = std::abs(ranges_[ranges_valid_index[i-1]] - ranges_[ranges_valid_index[i]])*(ranges_valid_index[i]-ranges_valid_index[i-1]);
                    float d2 = std::abs(ranges_[ranges_valid_index[i+1]] - ranges_[ranges_valid_index[i]])*(ranges_valid_index[i+1] - ranges_valid_index[i]);
                    float w = 0.5f/(ranges_valid_index[i+1]-ranges_valid_index[i-1]);

                    r = (1.0 -w )*ranges_[ranges_valid_index[i]] + w*(d2*ranges_[ranges_valid_index[i-1]] + d1*ranges_[ranges_valid_index[i+1]]   )/(d1+d2);
                    (ranges_[ranges_valid_index[i-1]] +ranges_[ranges_valid_index[i]]+ranges_[ranges_valid_index[i+1]] );
                    local_xy_points[i+i-2] = r*cos_angle_buffer[j];
                    local_xy_points[i+i-1] = r*sin_angle_buffer[j];
                    intensities_vec[i-1] = scan_intensities[j];

                }
                range_valid_num -= 2;
#endif
#if 0
                for(int i = 0 ; i < range_valid_num ;i++){
                    int j = ranges_valid_index[i];
                    float r = ranges_[j];
                    local_xy_points[i+i] = r*cos_angle_buffer[j];
                    local_xy_points[i+i +1] = r*sin_angle_buffer[j];
                }
//                range_valid_num++;

#endif



            }

        }

        // filter ranges
        void getLocalPoints(  const  std::vector<float>& scan_ranges, float angle_min, float  angle_increment, float range_min, float range_max){

            int N = scan_ranges.size();

//            ranges =  std::remove_const_t<std::vector<float>>(scan_ranges)  ;
             auto & ranges_ =   scan_ranges  ;

//            intensities = std::remove_const_t<std::vector<float>>(scan_intensities)  ;


            if (!init_done){


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
                arc_inc = sin(angle_increment);

                ranges_filtered.resize(N,scan_range_max); //scan_range_max
                ranges_valid_index.resize(N);
                init_done = true;

            }



            {

                range_valid_num = 0;

                bool valid = false;
                int offset = 3;
//                std::cout << "\n***********check range_valid_num\n";
                for(int i = offset ; i < N-offset;i++){
                    ranges_valid_index[range_valid_num] = i;
//                    std::cout << ranges_[i] << ", ";
                    valid = std::isnormal(ranges_[i])  &&(ranges_[i] < range_max) && (ranges_[i] > range_min)
                            && (  (std::abs(ranges_[i]-ranges_[i-1]) < scan_max_jump)|| (std::abs(ranges_[i]-ranges_[i+1]) < scan_max_jump) )
                            && ( std::abs( ranges_[i+1] - ranges_[i-1] ) < scan_max_jump)
                            && ( std::atan2( offset*arc_inc * ranges_[i],std::abs(ranges_[i+1] - ranges_[i-1])    ) > scan_noise_angle )
                            ;

                    range_valid_num += valid;

                }
//                range_valid_num++;

//                std::cout << "\n***********check range_valid_num : " << range_valid_num << std::endl;

#if 1
                for(int i = 1 ; i < range_valid_num-1;i++){
                    int j = ranges_valid_index[i];
                    float r = ranges_[j];
                    float d1 = std::abs(ranges_[ranges_valid_index[i-1]] - ranges_[ranges_valid_index[i]])*(ranges_valid_index[i]-ranges_valid_index[i-1]);
                    float d2 = std::abs(ranges_[ranges_valid_index[i+1]] - ranges_[ranges_valid_index[i]])*(ranges_valid_index[i+1] - ranges_valid_index[i]);
                    float w = 0.5f/(ranges_valid_index[i+1]-ranges_valid_index[i-1]);

                    r = (1.0 -w )*ranges_[ranges_valid_index[i]] + w*(d2*ranges_[ranges_valid_index[i-1]] + d1*ranges_[ranges_valid_index[i+1]]   )/(d1+d2);
                     (ranges_[ranges_valid_index[i-1]] +ranges_[ranges_valid_index[i]]+ranges_[ranges_valid_index[i+1]] );
                    local_xy_points[i+i-2] = r*cos_angle_buffer[j];
                    local_xy_points[i+i-1] = r*sin_angle_buffer[j];

                }
                range_valid_num -= 2;
#endif
#if 0
                for(int i = 0 ; i < range_valid_num ;i++){
                    int j = ranges_valid_index[i];
                    float r = ranges_[j];
                    local_xy_points[i+i] = r*cos_angle_buffer[j];
                    local_xy_points[i+i +1] = r*sin_angle_buffer[j];
                }
//                range_valid_num++;

#endif



            }

        }
        // filter ranges
        // add box noise filter
        void getLocalPoints(  const  std::vector<float>& scan_ranges, float angle_min, float  angle_increment, float range_min, float range_max,float filter_angle_min, float filter_angle_max){

            int N = scan_ranges.size();

//            ranges =  std::remove_const_t<std::vector<float>>(scan_ranges)  ;
            auto & ranges_ =   scan_ranges  ;

//            intensities = std::remove_const_t<std::vector<float>>(scan_intensities)  ;


            if (!init_done){


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
                arc_inc = sin(angle_increment);

                ranges_filtered.resize(N,scan_range_max); //scan_range_max
                ranges_valid_index.resize(N);
                init_done = true;

            }



            {

                range_valid_num = 0;

                bool valid = false;
                int offset = 3;
//                std::cout << "\n***********check range_valid_num\n";
                for(int i = offset ; i < N-offset;i++){
                    ranges_valid_index[range_valid_num] = i;
//                    std::cout << ranges_[i] << ", ";
                    valid = std::isnormal(ranges_[i])  &&(ranges_[i] < range_max) && (ranges_[i] > range_min)
                            &&  (angle_buffer[i] > filter_angle_min) &&  (angle_buffer[i] < filter_angle_max)
                            && (  (std::abs(ranges_[i]-ranges_[i-1]) < scan_max_jump)|| (std::abs(ranges_[i]-ranges_[i+1]) < scan_max_jump) )
                            && ( std::abs( ranges_[i+1] - ranges_[i-1] ) < scan_max_jump)
                            && ( std::atan2( offset*arc_inc * ranges_[i],std::abs(ranges_[i+1] - ranges_[i-1])    ) > scan_noise_angle )
                             ;

                    range_valid_num += valid;

                }
//                range_valid_num++;

//                std::cout << "\n***********check range_valid_num : " << range_valid_num << std::endl;

#if 1
                for(int i = 1 ; i < range_valid_num-1;i++){
                    int j = ranges_valid_index[i];
                    float r = ranges_[j];
                    float d1 = std::abs(ranges_[ranges_valid_index[i-1]] - ranges_[ranges_valid_index[i]])*(ranges_valid_index[i]-ranges_valid_index[i-1]);
                    float d2 = std::abs(ranges_[ranges_valid_index[i+1]] - ranges_[ranges_valid_index[i]])*(ranges_valid_index[i+1] - ranges_valid_index[i]);
                    float w = 0.5f/(ranges_valid_index[i+1]-ranges_valid_index[i-1]);

                    r = (1.0 -w )*ranges_[ranges_valid_index[i]] + w*(d2*ranges_[ranges_valid_index[i-1]] + d1*ranges_[ranges_valid_index[i+1]]   )/(d1+d2);
                    (ranges_[ranges_valid_index[i-1]] +ranges_[ranges_valid_index[i]]+ranges_[ranges_valid_index[i+1]] );
                    local_xy_points[i+i-2] = r*cos_angle_buffer[j];
                    local_xy_points[i+i-1] = r*sin_angle_buffer[j];

                }
                range_valid_num -= 2;
#endif
#if 0
                for(int i = 0 ; i < range_valid_num ;i++){
                    int j = ranges_valid_index[i];
                    float r = ranges_[j];
                    local_xy_points[i+i] = r*cos_angle_buffer[j];
                    local_xy_points[i+i +1] = r*sin_angle_buffer[j];
                }
//                range_valid_num++;

#endif



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

            std::cout << "start filterMarker\n";
            std::cout << "N : " << N <<std::endl;
            std::cout << "ranges.size() : " << ranges.size() <<std::endl;
            std::cout << "ranges_filtered.size() : " << ranges_filtered.size() <<std::endl;

            filter_cluster_range.clear();
            filter_cluster_index.resize(N);
            filter_cluster_points.resize(N);

            filter_markers = markers;

            for(int i = 0; i < N;i++){
                filter_cluster_index[i].clear();

                float m_angle = atan2(markers[i][1],markers[i][0]);
                float m_range = sqrt(markers[i][0]*markers[i][0] + markers[i][1]*markers[i][1]);
                float angle_offset = asin(filterOption.radius/m_range);

                filter_markers[i][0] = cos(m_angle)*(m_range- filterOption.marker_radius);
                filter_markers[i][1] = sin(m_angle)*(m_range- filterOption.marker_radius);


                auto lb = std::lower_bound(angle_buffer.begin(),angle_buffer.end(),m_angle-angle_offset);
                auto ub = std::upper_bound(angle_buffer.begin(),angle_buffer.end(),m_angle+angle_offset);

                int lb_index = (m_angle-angle_offset - angle_buffer[0])*scan_angle_inc_inv;
                int ub_index = (m_angle+angle_offset - angle_buffer[0])*scan_angle_inc_inv;
                ub_index = ub_index>=angle_buffer.size() ? angle_buffer.size()-1:ub_index;

                int m_index = 0.5*(lb_index + ub_index);


//                std::cout << "m_angle: " << m_angle << ", m_range: " << m_range << ", angle_offset: " << angle_offset << std::endl;
//                std::cout << "lb: " << std::distance(angle_buffer.begin(), lb) << "ub: " << std::distance(angle_buffer.begin(), ub) << std::endl;
//                std::cout << "lb_index: " << lb_index  << "ub_index: " << ub_index<< std::endl;

                int point_num = ub_index - lb_index;
//                std::cout << "point_num: " << point_num  << ", filterOption.min_light_num: " << filterOption.min_light_num << ", filterOption.max_light_num: " << filterOption.max_light_num  << ", filterOption.min_intensity: " << filterOption.min_intensity << ", filterOption.radius: " << filterOption.radius  << std::endl;

                float intensity_thresh = filterOption.min_intensity;

                float intensities_array[100];

                if( point_num>=filterOption.min_light_num ){


                    if(lb_index>=0){
                        for(int j =lb_index; j < ub_index -1 ;j++){
//                            std::cout << "check j : " << j<< ", " <<  ranges[j] << ", " << global_xy_points_vec[j][2]  << std::endl;

                            if(  (fabs(ranges[j] - m_range) < filterOption.radius)  && (fabs(ranges[j] - ranges[j+1] ) < 0.1) &&( global_xy_points_vec[j][2] > intensity_thresh   ) ){
                                filter_cluster_index[i].push_back(j);
                            }
                        }
                        {
                            int j = ub_index -1;
                            if(  (fabs(ranges[j] - m_range) < filterOption.radius)  && (fabs(ranges[j] - ranges[j-1] ) < 0.1) &&( global_xy_points_vec[j][2] > intensity_thresh   ) ){
                                filter_cluster_index[i].push_back(j);
                            }
                        }


                    }else{
                        for(int j =0; j < ub_index-1;j++){
//                            std::cout << "check j : " << j<< ", " <<  ranges[j] << ", " << global_xy_points_vec[j][2]  << std::endl;

                            if(  (fabs(ranges[j] - m_range) < filterOption.radius)  && (fabs(ranges[j] - ranges[j+1] ) < 0.1) &&( global_xy_points_vec[j][2] > intensity_thresh   ) ){
                                filter_cluster_index[i].push_back(j);
                            }
                        }
                        {
                            int j = ub_index -1;
                            if(  (fabs(ranges[j] - m_range) < filterOption.radius)  && (fabs(ranges[j] - ranges[j-1] ) < 0.1) &&( global_xy_points_vec[j][2] > intensity_thresh   ) ){
                                filter_cluster_index[i].push_back(j);
                            }
                        }

                        for(int j =ranges.size() + lb_index; j < ranges.size() - 1;j++){
//                            std::cout << "check j : " << j<< ", " <<  ranges[j] << ", " << global_xy_points_vec[j][2]  << std::endl;

                            if(  (fabs(ranges[j] - m_range) < filterOption.radius)  && (fabs(ranges[j] - ranges[j+1] ) < 0.1) &&( global_xy_points_vec[j][2] > intensity_thresh   ) ){
                                filter_cluster_index[i].push_back(j);
                            }
                        }
                        {
                            int j = ranges.size()  -1;
                            if(  (fabs(ranges[j] - m_range) < filterOption.radius)  && (fabs(ranges[j] - ranges[j-1] ) < 0.1) &&( global_xy_points_vec[j][2] > intensity_thresh   ) ){
                                filter_cluster_index[i].push_back(j);
                            }
                        }

                    }

//                    std::cout << "==  init  filter_cluster_index[i].size = " << filter_cluster_index[i].size() << std::endl;


                    int k =   filterOption.max_light_num >= filter_cluster_index[i].size()   ? filter_cluster_index[i].size():  filterOption.max_light_num;

                   std::nth_element(filter_cluster_index[i].begin(),filter_cluster_index[i].begin() + k, filter_cluster_index[i].end(),[&](auto& v1, auto &v2){
                        return global_xy_points_vec[v1][2] > global_xy_points_vec[v2][2];
                    });
                    filter_cluster_index[i].resize(k);

//                    std::cout << "==  final filter_cluster_index[i].size = " << filter_cluster_index[i].size() << std::endl;



                    filter_cluster_range.emplace_back(std::array<int,2>{lb_index,ub_index});
                }


            }


//            ranges_filtered.resize(ranges.size(), scan_range_max);
            std::fill(ranges_filtered.begin(), ranges_filtered.end(),scan_range_max);


#if 0
            for(int i = 0; i < N;i++){
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
            for(int i = 0; i < N;i++){
                filter_cluster_points[i].clear();
                filter_cluster_points[i].emplace_back(std::array<float,3>{0.0,0.0,0.0});

                for(auto& j :filter_cluster_index[i]){
                    filter_cluster_points[i].emplace_back(global_xy_points_vec[j]);
                    filter_cluster_points[i][0][0] += global_xy_points_vec[j][0];
                    filter_cluster_points[i][0][1] += global_xy_points_vec[j][1];

                }
            }
            for(int i = 0; i < N;i++) {
                filter_cluster_points[i][0][0] /=  filter_cluster_points[i].size() > 1 ? (filter_cluster_points[i].size() -1):1.0;
                filter_cluster_points[i][0][1] /= filter_cluster_points[i].size() > 1 ? (filter_cluster_points[i].size() -1):1.0;
            }

                // pattern match
            // first find closest and best matching
            //
            for(int i = 0; i < N;i++){
                int valid_egge_num = 0;
                if(filter_cluster_points[i].size() >1){
                    for(int j = 0; j < N;j++){
                        if (i == j)  continue;

                        //
                        float dist =

                                std::abs(sqrt(   (markers[i][0] - markers[j][0] ) *(markers[i][0] - markers[j][0] ) + (markers[i][1] - markers[j][1] ) *(markers[i][1] - markers[j][1] ))
                                         - sqrt(  (filter_cluster_points[i][0][0] - filter_cluster_points[j][0][0])*(filter_cluster_points[i][0][0] - filter_cluster_points[j][0][0]) +  (filter_cluster_points[i][0][1] - filter_cluster_points[j][0][1])*(filter_cluster_points[i][0][1] - filter_cluster_points[j][0][1])  )
                                )

                        ;
                        if(dist < filterOption.edge_offset){
                            valid_egge_num ++;

                        }

                    }
                }

                if (valid_egge_num < filterOption.min_edge_num){
                    filter_cluster_points[i].clear();

                }
            }




            std::cout << "filterMarker Done\n";
//            ranges_filtered = ranges;
        }


    };
}

#endif //SCAN_REPUBLISHER_LASER_SCAN_H
