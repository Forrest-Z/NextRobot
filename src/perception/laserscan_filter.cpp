//
// Created by waxz on 23-2-4.
//

#include <iostream>
#include "laserscan_filter.h"
#include "normal_estimation_2d.h"
#include "math/math_basic.h"

namespace perception{



    void LaserScanBoxFilter(const std::vector<float> &scan_points, int point_num,
                                   std::vector<geometry::Point>& filter_points, int& filter_point_num,
                                   float min_x, float max_x, float min_y, float max_y
    ) {

        filter_point_num = 0;
        bool valid = false;
        filter_points.resize(point_num);
        for (int i = 0; i < point_num; i++) {
            valid = scan_points[i + i]> min_x && scan_points[i + i] < max_x && scan_points[i + i + 1] > min_y && scan_points[i + i + 1] < max_y;
            filter_points[filter_point_num].x = scan_points[i + i];
            filter_points[filter_point_num].y = scan_points[i + i + 1];
            filter_points[filter_point_num].r = std::sqrt(scan_points[i + i + 1]*scan_points[i + i + 1] + scan_points[i + i]*scan_points[i + i]);

            filter_point_num += valid;

        }

    }

    void LaserScanBoxFilter(const std::vector<float> &scan_points_laser, const std::vector<float> &scan_points_base, int point_num,
                            std::vector<geometry::Point>& filter_points_laser, std::vector<geometry::Point>& filter_points_base, int& filter_point_num,
                            float min_x, float max_x, float min_y, float max_y
    ) {

        filter_point_num = 0;
        bool valid = false;
        filter_points_base.resize(point_num);
        filter_points_laser.resize(point_num);

        for (int i = 0; i < point_num; i++) {
            valid = scan_points_base[i + i]> min_x && scan_points_base[i + i] < max_x && scan_points_base[i + i + 1] > min_y && scan_points_base[i + i + 1] < max_y;
            filter_points_base[filter_point_num].x = scan_points_base[i + i];
            filter_points_base[filter_point_num].y = scan_points_base[i + i + 1];
            filter_points_base[filter_point_num].r = std::sqrt(scan_points_base[i + i + 1]*scan_points_base[i + i + 1] + scan_points_base[i + i]*scan_points_base[i + i]);

            filter_points_laser[filter_point_num].x = scan_points_laser[i + i];
            filter_points_laser[filter_point_num].y = scan_points_laser[i + i + 1];
            filter_points_laser[filter_point_num].r = std::sqrt(scan_points_laser[i + i + 1]*scan_points_laser[i + i + 1] + scan_points_laser[i + i]*scan_points_laser[i + i]);


            filter_point_num += valid;

        }
//    filter_point_num ++;


    }

    void LaserScanRadiusFilter(const std::vector<float> &scan_points_laser, const std::vector<float> &scan_points_base, int point_num,
                               std::vector<geometry::Point>& filter_points_laser, std::vector<geometry::Point>& filter_points_base, int& filter_point_num,
                               float relative_x, float relative_y, float filter_radius
    ) {

        filter_point_num = 0;
        bool valid = false;
        filter_points_base.resize(point_num);
        filter_points_laser.resize(point_num);

        filter_radius *= filter_radius;

        for (int i = 0; i < point_num; i++) {
            valid = ((scan_points_base[i + i] - relative_x)*(scan_points_base[i + i] - relative_x) + ( scan_points_base[i + i + 1] - relative_y)*( scan_points_base[i + i + 1] - relative_y)) < filter_radius;
            filter_points_base[filter_point_num].x = scan_points_base[i + i];
            filter_points_base[filter_point_num].y = scan_points_base[i + i + 1];
            filter_points_base[filter_point_num].r = std::sqrt(scan_points_base[i + i + 1]*scan_points_base[i + i + 1] + scan_points_base[i + i]*scan_points_base[i + i]);
            filter_points_base[filter_point_num].b = std::atan2(scan_points_base[i + i + 1], scan_points_base[i + i]);
            filter_points_laser[filter_point_num].x = scan_points_laser[i + i];
            filter_points_laser[filter_point_num].y = scan_points_laser[i + i + 1];
            filter_points_laser[filter_point_num].r = std::sqrt(scan_points_laser[i + i + 1]*scan_points_laser[i + i + 1] + scan_points_laser[i + i]*scan_points_laser[i + i]);
            filter_points_laser[filter_point_num].b = std::atan2(scan_points_laser[i + i + 1], scan_points_laser[i + i]);

            filter_points_base[filter_point_num].b = angle_normalise(filter_points_base[filter_point_num].b,filter_points_base[0].b);
            filter_points_laser[filter_point_num].b = angle_normalise(filter_points_laser[filter_point_num].b,filter_points_laser[0].b);

//            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  " << scan_points_base[i + i] << ", " << scan_points_base[i + i + 1] << std::endl;  ;
//            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  " << scan_points_laser[i + i] << ", " << scan_points_laser[i + i + 1] << std::endl;  ;

            filter_point_num += valid;

        }
//    filter_point_num ++;


    }
/*
 find
 */
    void LaserScanSegment( const std::vector<geometry::Point>& filter_points, std::vector<std::vector<geometry::Point>>& segments, int filter_point_num, float split_dist_x,float split_dist_y,float split_dist_r, int min_point_in_seg_num, float shadow_angle ) {


        segments.clear();

        if(filter_points.size() < min_point_in_seg_num){
            return;
        }
        // split to segments
        std::vector<geometry::Point> segment;
        bool process_done = false;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " start process " << std::endl;  ;

        for(int i = 1 ; i < filter_point_num; i++){
            bool split = std::abs(filter_points[i].x - filter_points[i -1].x) > split_dist_x
                         || std::abs(filter_points[i].y - filter_points[i -1].y) > split_dist_y
                         || std::abs(filter_points[i].r - filter_points[i -1].r) > split_dist_r;

            if(split || (i == filter_point_num-1)){

                for(int j = 0 ; j < filter_point_num ;j++){
                    int l =  (i+j)%filter_point_num;
                    int l_pre = (l-1)%filter_point_num;
                    l_pre = l_pre<0 ? l_pre+ filter_point_num:l_pre;

//                std::cout << "j = " << j << ", l = " << l << ", " << filter_points[l].x << ", " << filter_points[l].y <<"\n ";
//                std::cout << "check l_pre = " << l_pre << ", " << filter_points[l_pre].x << ", " << filter_points[l_pre].y <<"\n ";

                    split = std::abs(filter_points[l].x - filter_points[l_pre].x) > split_dist_x
                            || std::abs(filter_points[l].y - filter_points[l_pre].y) > split_dist_y
                            || std::abs(filter_points[l].r - filter_points[l_pre].r) > split_dist_r;
//                std::cout << "check dist = " << std::abs(filter_points[l].x - filter_points[l_pre].x)  << ", "  << std::abs(filter_points[l].y - filter_points[l_pre].y) <<", "<< std::abs(filter_points[l].r - filter_points[l_pre].r) <<"\n"    ;


                    if(split|| (j == filter_point_num-1)){
//                    std::cout << "====== split,  l = " << l << ", j = " << j << ", filter_point_num = " << filter_point_num <<"\n ";
#if 0
                        if((j == filter_point_num-1)){
                        segment.emplace_back(filter_points[l]);
                    }
#endif
                        if(segment.size() > min_point_in_seg_num){
                            segments.emplace_back(segment);
                        }

                        segment.clear();


                    }

                    segment.emplace_back(filter_points[l]);

                }

                process_done = true;

            }
            if(process_done){
                break;
            }
        }

        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " start meager " << std::endl;  ;


        float meager_dist = split_dist_x*split_dist_x;
        for(int i = 0 ; i < segments.size();i++){
            if( segments[i].empty()) continue;
            for(int j = i+1 ; j < segments.size();j++){
                if( segments[j].empty()) continue;

                if(segments[i].back().SquareDist(segments[j].front())<meager_dist || segments[j].back().SquareDist(segments[i].front())<meager_dist ){
                    segments[i].insert(segments[i].end(),segments[j].begin(), segments[j].end());
                    segments[j].clear();
                }

            }
        }
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " finish meager " << std::endl;  ;



        // remove shadow
        // compute normal form two side
        NormalEst2d normalEst2d;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " start remove shadow " << std::endl;  ;


        float nx, ny, nz, curvature;
        bool is_shadow = false;
        int check_shadow_num = 4;
        int detect_shadow_num = 0;

        for(int i = 0 ; i < segments.size();i++){

            auto& s = segments[i];
            std::sort(s.begin(),s.end(),[](auto& v1, auto& v2){
                return v1.r < v2.r;
            });
            int N = s.size();
            if(N  <check_shadow_num + check_shadow_num){
                continue;
            }

            auto& front_point = s.front();
            auto& back_point = s.back();

            {
                normalEst2d.reset();

                normalEst2d.addCenter(front_point.x,front_point.y );

                for(int j = 0 ; j < check_shadow_num; j++){
                    auto& p = s[j];
                    normalEst2d.addPoints(p.x,p.y);

                }
                normalEst2d.compute(nx,ny,nz,curvature);
                std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", " << nx <<  ", " << ny << ", " << nz << ", " << curvature << "\n";

                is_shadow = curvature > shadow_angle;
                detect_shadow_num = 0;
                int k = check_shadow_num;
                while(is_shadow && k < N ){
                    auto& p = s[k];
                    normalEst2d.addPoints(p.x,p.y);
                    normalEst2d.compute(nx,ny,nz,curvature);
                    std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", " << nx <<  ", " << ny << ", " << nz << ", " << curvature << "\n";

                    k ++;
                    is_shadow = curvature > shadow_angle;
                    detect_shadow_num += is_shadow;
                }

                if(detect_shadow_num >0){
                    detect_shadow_num += check_shadow_num;
                    for(int j = 0 ; j < detect_shadow_num ; j++){
                        auto& p = s[j];
                        p.i = -2;
                    }
                }

            }
            {
                normalEst2d.reset();
                normalEst2d.addCenter(back_point.x,back_point.y );
                for(int j = 0 ; j < check_shadow_num; j++){
                    auto& p = s[N-j-1];
                    normalEst2d.addPoints(p.x,p.y);

                }
                normalEst2d.compute(nx,ny,nz,curvature);
                std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", " << nx <<  ", " << ny << ", " << nz << ", " << curvature << "\n";

                is_shadow = curvature > shadow_angle;
                detect_shadow_num = 0;
                int k = check_shadow_num;
                while(is_shadow && k < N ){
                    auto& p = s[N-k-1];
                    normalEst2d.addPoints(p.x,p.y);
                    normalEst2d.compute(nx,ny,nz,curvature);
                    std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", " << nx <<  ", " << ny << ", " << nz << ", " << curvature << "\n";

                    k ++;
                    is_shadow = curvature >shadow_angle;
                    detect_shadow_num += is_shadow;
                }
                if(detect_shadow_num >0){
                    detect_shadow_num += check_shadow_num;
                    for(int j = 0 ; j < detect_shadow_num ; j++){
                        auto& p = s[N-j-1];
                        p.i = -2;
                    }
                }
            }


            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << "segment size before remove shadow " << s.size() << "\n";


            s.erase(std::remove_if(s.begin(),
                                      s.end(),
                                      [](auto& p){ return p.i == -2; }),
                       s.end());
            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << "segment size after remove shadow " << s.size() << "\n";

            if(!s.empty()){
                std::sort(s.begin(),s.end(), [](auto& v1, auto& v2){ return v1.b < v2.b; });
            }


        }
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " finish remove shadow " << std::endl;  ;

#if 1

        std::cout << "check segments:\n";
        for(auto &s : segments){
            std::cout << "\ncheck segments points: " << s.size() << "\n";

            for(auto& e: s){
                std::cout <<"[" << e.x << ", " << e.y << ", " << e.r << "], ";
            }

        }

#endif
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " finish all process " << std::endl;  ;



    }



}