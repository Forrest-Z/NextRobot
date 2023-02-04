//
// Created by waxz on 23-1-4.
//

#include <fstream>


#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "xmlrpcpp/XmlRpc.h"



#include "nlohmann/json.hpp"
#include <plog/Log.h> // Step1: include the headers
#include "plog/Initializers/RollingFileInitializer.h"
#include "plog/Appenders/ColorConsoleAppender.h"


#include "common/task.h"
#include "common/suspend.h"
#include "common/clock_time.h"
#include "common/subprocess.hpp"



#include "sensor/laser_scan.h"
#include "sensor/geometry_types.h"
#include "sensor/pointcloud_ros.h"

#include "perception/pointcloud_detection.h"


#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

//#include <autodiff/forward/real.hpp>
//#include <autodiff/forward/real/eigen.hpp>

#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>
/*


 */

#include "pid.h"


autodiff::var
circle_opt_fnd(const autodiff::ArrayXvar& x, const std::vector<geometry::Point>& points,int point_num, float radius_2)
{
    autodiff::var r = 0.0;
    autodiff::var t = 0.0;

    for(int i = 0 ; i < point_num ;i++){
        t = (x(0) - points[i].x)*(x(0) - points[i].x) + (x(1) - points[i].y)*(x(1) - points[i].y) - radius_2;
        r += t*t;
    }
    return r;
}

struct CircleCostFunction{

    const std::vector<geometry::Point>& points;
    int point_num;
    float radius = 0.1;
    float radius_2 = radius;


    CircleCostFunction(const std::vector<geometry::Point>& t_points, int t_point_num, float t_radius):points(t_points),point_num(t_point_num), radius(t_radius),radius_2(radius*radius){
    }



    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
    {

        autodiff::ArrayXvar xd = x.eval();

        autodiff::var y = circle_opt_fnd(xd,points,point_num,radius_2);

        if (grad_out) {
            Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

            *grad_out = grad_tmp;
        }

        return autodiff::val(y);
    }
};


/*
 box filter
 remove points outside of box
 */
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
//        std::cout << "** " << scan_points_base[i + i] << ", " << scan_points_base[i + i + 1] << ", " << valid << "\n";
        filter_points_base[filter_point_num].x = scan_points_base[i + i];
        filter_points_base[filter_point_num].y = scan_points_base[i + i + 1];
        filter_points_base[filter_point_num].r = std::sqrt(scan_points_base[i + i + 1]*scan_points_base[i + i + 1] + scan_points_base[i + i]*scan_points_base[i + i]);
        filter_points_base[filter_point_num].b = std::atan2(scan_points_base[i + i + 1], scan_points_base[i + i]);
        filter_points_laser[filter_point_num].x = scan_points_laser[i + i];
        filter_points_laser[filter_point_num].y = scan_points_laser[i + i + 1];
        filter_points_laser[filter_point_num].r = std::sqrt(scan_points_laser[i + i + 1]*scan_points_laser[i + i + 1] + scan_points_laser[i + i]*scan_points_laser[i + i]);
        filter_points_laser[filter_point_num].b = std::atan2(scan_points_laser[i + i + 1], scan_points_laser[i + i]);


        filter_point_num += valid;

    }
//    filter_point_num ++;


}



/*
 find
 */
void LaserScanSegment( const std::vector<geometry::Point>& filter_points, std::vector<std::vector<geometry::Point>>& segments, int filter_point_num, float split_dist_x,float split_dist_y,float split_dist_r, int min_point_in_seg_num
) {


    segments.clear();

    if(filter_points.size() < min_point_in_seg_num){
        return;
    }
    // split to segments
    std::vector<geometry::Point> segment;
    bool process_done = false;
//    PLOGD << "filter_point_num " << filter_point_num << std::endl;

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
                        segments.emplace_back(std::move(segment));
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


    PLOGD << "init segments.size() " << segments.size() << std::endl;

    float meager_dist = split_dist_x*split_dist_x;
    for(int i = 0 ; i < segments.size();i++){
        for(int j = i+1 ; j < segments.size();j++){

            if(segments[i].back().SquareDist(segments[j].front())<meager_dist || segments[j].back().SquareDist(segments[i].front())<meager_dist ){
                segments[i].insert(segments[i].end(),segments[j].begin(), segments[j].end());
                segments[j].clear();
            }

        }
    }
#if 1

    std::cout << "check segments:\n";
    for(auto &s : segments){
        std::cout << "\ncheck segments points: " << s.size() << "\n";

        for(auto& e: s){
            std::cout <<"[" << e.x << ", " << e.y << ", " << e.r << "], ";
        }

    }
#endif

    PLOGD << "get segments num " << segments.size() << std::endl;

//    std::sort(filter_points.begin(),filter_points.end(), [](auto& v1, auto& v2){ return v1.r < v2.r; });
    // find center angle
    // find mean edge point




}

//float NormaliseAngle(float angle, float mean){
//    while( std::abs(angle - mean) > M_PI)
//}

// find circle
void FindCircle(std::vector<geometry::Point>& points,float radius, float edge_radius, float edge_range_offset, int min_point_num, float & center_x, float & center_y, float& error_mean){

    if(points.size() < min_point_num){
        PLOGD << "check points.size() fail return : " << points.size()  << std::endl;

        error_mean = 100.0;
        return;
    }
    std::sort(points.begin(),points.end(), [](auto& v1, auto& v2){ return v1.r < v2.r; });


    float angle_mean = std::atan2(points[0].y,points[0].x ),angle_sum = 0.0;
//    PLOGD << "check angle_mean: " << angle_mean << std::endl;

    int valid_num = 0;
    float min_r = points[0].r -0.02, max_r = points[0].r + edge_radius;

//    std::cout << "compute center angle:\n";
    float normalise_angle = 0.0;
    for(auto&p : points){
        normalise_angle = std::abs(p.b - angle_mean) < M_PI ? p.b : (p.b + ((p.b - angle_mean) > 0.0 ? -M_PI*2: M_PI*2) );
//        std::cout << "angle: " << p.b <<", normalise_angle: " << normalise_angle << "\n";

        if( (p.r < max_r )&&(p.r > min_r) ){
            angle_sum += normalise_angle;
            valid_num++;

        }else{
//            break;

        }
    }

    if(valid_num < min_point_num){
        PLOGD << "check valid_num fail return" << std::endl;

        error_mean = 100.0;
        return;
    }


    angle_mean = angle_sum/float(valid_num);
    PLOGD << "valid_num: " << valid_num <<"angle_mean "  << angle_mean   << std::endl;

    float ux = std::cos(angle_mean);
    float uy = std::sin(angle_mean);


    float edge_range = 0.0;
    int egde_num = 0;
    float angle_offset = std::atan2(edge_range_offset, points[0].r);
    PLOGD << "angle_offset: " << angle_offset <<", angle_mean "  << angle_mean   << std::endl;

    for(auto&p : points){
        normalise_angle = std::abs(p.b - angle_mean) < M_PI ? p.b : (p.b + ((p.b - angle_mean) > 0.0 ? -M_PI*2: M_PI*2) );
//        std::cout << "angle: " << p.b <<", normalise_angle: " << normalise_angle << "\n";

        if(std::abs(normalise_angle - angle_mean)<angle_offset){
            edge_range += p.r;
            egde_num++;
        }else{
//            break;

        }
    }
    PLOGD << "check egde_num: " << egde_num << std::endl;

    if(egde_num < min_point_num){
        PLOGD << "check egde_num fail return" << std::endl;

        error_mean = 100.0;
        return;
    }

    edge_range /= float(egde_num);

    edge_range += radius;
    PLOGD << "check edge_range: " << edge_range << std::endl;


    float cx = ux*(edge_range);
    float cy = uy*(edge_range);

    center_x = ux*(edge_range);
    center_y = uy*(edge_range);

//    cx = points[0].x + ux*(radius);
//    cy = points[0].y + uy*(radius);

    float error_sum = 0.0 ;


    error_sum = 0.0;
//    std::cout << "check error:\n";
    for(int i = 0 ; i <points.size();i++){
        auto& p = points[i];
        float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
//        std::cout << d << ", ";
        error_sum += std::abs(d- radius) ;
    }
    error_mean = error_sum/float(points.size());
    PLOGD << "check cx: " << cx << ", cy: " << cy << std::endl;

    PLOGD << "check error_mean: " << error_mean << std::endl;



    Eigen::VectorXd x(2);
    x << cx, cy;

    optim::algo_settings_t settings;
//    settings.iter_max = 20;
//    settings.bfgs_settings.wolfe_cons_1 = 1e-4;
//    settings.bfgs_settings.wolfe_cons_2 = 0.8;

//    settings.print_level = 1;

    settings.vals_bound = true;

    settings.lower_bounds = optim::ColVec_t::Zero(2);
    settings.lower_bounds(0) = cx - 0.03;
    settings.lower_bounds(1) = cy - 0.03;

    settings.upper_bounds = optim::ColVec_t::Zero(2);
    settings.upper_bounds(0) = cx + 0.03;
    settings.upper_bounds(1) = cy + 0.03;

    CircleCostFunction opt_fn_obj(points,valid_num, radius) ;

    bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "solution: x = \n" << x << std::endl;

    cx = x(0);
    cy = x(1);

    error_sum = 0.0;
    for(int i = 0 ; i <points.size();i++){
        auto& p = points[i];
        float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
//        std::cout << d << ", ";
        error_sum += std::abs(d- radius) ;
    }
    error_mean = error_sum/float(points.size());
    PLOGD << "check cx: " << cx << ", cy: " << cy << std::endl;

    PLOGD << "check error_mean: " << error_mean << std::endl;
    center_x = cx;
    center_y = cy;
    return  ;



}




#if 0
// The multi-variable function for which derivatives are needed
autodiff::real f(autodiff::real x, autodiff::real y, autodiff::real z)
{
    return 1 + x + y + z + x*y + y*z + x*z + x*y*z + exp(x/y + y/z);
}
// The multi-variable function for which derivatives are needed
autodiff::var f2(autodiff::var x, autodiff::var y, autodiff::var z)
{
    return 1 + x + y + z + x*y + y*z + x*z + x*y*z + exp(x/y + y/z);
}
#endif

autodiff::var
opt_fnd(const autodiff::ArrayXvar& x)
{
    return (x(0) - 1.0)*(x(0) - 1.0)
    + (x(1) - 1.5)*(x(1) - 1.5)
      + (x(2) - 1.8)*(x(2) - 1.8)
    ;
}


double
opt_fn(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
{

    autodiff::ArrayXvar xd = x.eval();

    autodiff::var y = opt_fnd(xd);

    if (grad_out) {
        Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

        *grad_out = grad_tmp;
    }

    return autodiff::val(y);
}



void FindShelfLeg(){}


/*

 todo:
 1. center/radius filter, update with odom change
 2.

 */


/*
 detect target:
 1. one circle
 2. 4 legs of a shelf
 3. 2 legs of a shelf
 */



/*

 compute control points array, not direct rotation and forward cmd_vel
 two stage controller
 1.

 */
struct MotionControl{

};

int main(int argc, char** argv){


    plog::RollingFileAppender<plog::CsvFormatter> fileAppender("scan_circle.csv", 800000, 10); // Create the 1st appender.
    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.



    //==== ros
    ros::init(argc, argv, "scan_matching");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");




    std::string mode = "circle";
    const char* mode_param = "mode";
    const char* MODE_CIRCLE = "circle";
    const char* MODE_SHELF4 = "shelf4";
    const char* MODE_SHELF2 = "shelf2";


    // target in base_frame
    // x, y, yaw, init_radius, track_radius
    std::vector<float> detect_target_relative_pose{0.0,0.0,0.0,0.0,0.0};
    const char* detect_target_relative_pose_param = "detect_target_relative_pose";


    transform::Transform2d detect_target_relative_tf;
    transform::Transform2d detect_target_absolute_tf;
    bool detect_target_absolute_pose_computed = false;
    bool detect_target_stop_update = false;
    float detect_target_stop_update_dist = 0.3;
    const char* detect_target_stop_update_dist_param = "detect_target_stop_update_dist";

    std::array<float,4> shelf_empty_region{0.0,0.0,0.0,0.0};

    // odom_base_tf_1* detect_target_relative_tf_1 = odom_base_tf_2* detect_target_relative_tf_2
    // detect_target_relative_tf_2 = odom_base_tf_2.inverse()*odom_base_tf_1* detect_target_relative_tf_1

    transform::Transform2d odom_base_tf_1, odom_base_tf_2;


    // shelf len_x, len_y
    std::vector<float> shelf_len_y_x{0.0,0.0};
    std::vector<float> shelf_len_y_x_real{0.0,0.0};

    const char* shelf_len_y_x_param = "shelf_len_y_x";
    const char* shelf_len_y_x_real_param = "shelf_len_y_x_real";

    /*
     1. first detect shelf
     maybe 4 legs or 2 legs
     define shelf_center_pose_in_base, shelf_len_y, shelf_len_x, search_radius

     2. detect
     compute leg pose
     filter and segment
     find empty region doesn't contain points: shelf center, line center
     or big box cover robot ase and leg center

     match with shelf_len_y shelf_len_x

     find center line, equally divide L1_L2 L3_L4


     L1----------------------------L2


                   <> target_origin


     L3----------------------------L4


                   ^ x
                   |
                   |
          y<-------robot
     * */

    /*

     3. final control pose

     define
     distance: robot to L1_L2_center
     angle: angle between robot_x and shelf_origin_to_L1_L2_center


     4. control
     robot move to target pose without rotation
     check tolerance
     if not in tolerance, move back and move forward again
     if in tolerance, rotate to target angle


     L1-------------O---------------L2


                   ^ x
                   |
                   |
          y<-------robot



     L3----------------------------L4

     * */


    float front_radius = 0.03;
    const char* front_radius_param = "front_radius";
    int front_min_num = 5;
    const char* front_min_num_param = "front_min_num";



    std::vector<geometry::DetectTarget > shelf_leg_base_pose;
    std::vector<std::vector<geometry::DetectTarget>> shelf_leg_front_pose;
    std::vector<int> shelf_leg_important_index{0,1,2,3};
    std::vector< std::array<int,10>> shelf_leg_match_result;
    float pattern_match_radius = 0.05;
    const char* pattern_match_radius_param = "pattern_match_radius";
    int shelf_leg_min_matched_num = 4;
    float shelf_max_fit_error = 0.04;
    std::vector<float> shelf_rect_diff{0.05,0.05, 0.05}; // edge1_diff, edge2_diff, angle_diff
    const char* shelf_rect_diff_param = "shelf_rect_diff";



//    std::vector<geometry::DetectTarget > shelf_leg_laser_pose;


    float sleep_time = 50.0;
    const char* sleep_time_param = "sleep_time";

    float tf_wait_time = 0.08;
    const char* tf_wait_time_param = "tf_wait_time";

    float range_max = 30.0;
    const char* range_max_param = "range_max";
    float range_min = 1.0;
    const char* range_min_param = "range_min";

    float filer_angle_min = - 1.57;
    float filer_angle_max = 1.57;
    const char* filer_angle_min_param = "filer_angle_min";
    const char* filer_angle_max_param = "filer_angle_max";


    float scan_point_jump = 0.06;
    float scan_noise_angle = 0.06;
    const char* scan_point_jump_param = "scan_point_jump";
    const char* scan_noise_angle_param = "scan_noise_angle";

    float box_filter_min_x = 0.1;
    float box_filter_max_x = 1.2;

    float box_filter_min_y = -0.4;
    float box_filter_max_y = 0.4;

    const char* box_filter_min_x_param = "box_filter_min_x";
    const char* box_filter_max_x_param = "box_filter_max_x";
    const char* box_filter_min_y_param = "box_filter_min_y";
    const char* box_filter_max_y_param = "box_filter_max_y";


    float circle_radius = 0.1;
    float circle_edge_radius = 0.08;
    int circle_min_num_in_radius = 10;
    float circle_edge_range_offset = 0.03;
    int circle_min_point_num = 10;

    const char* circle_radius_param = "circle_radius";
    const char* circle_edge_radius_param = "circle_edge_radius";
    const char* circle_min_num_in_radius_param = "circle_min_num_in_radius";
    const char* circle_edge_range_offset_param = "circle_edge_range_offset";
    const char* circle_min_point_num_param = "circle_min_point_num";

    float max_fit_error = 0.01;
    const char* max_fit_error_param = "max_fit_error";

    float split_dist_x = 0.08;
    float split_dist_y = 0.08;
    float split_dist_r = 0.05;

    int min_point_num_in_seg = 10;
    const char* split_dist_x_param = "split_dist_x";
    const char* split_dist_y_param = "split_dist_y";
    const char* split_dist_r_param = "split_dist_r";

    const char* min_point_num_in_seg_param = "min_point_num_in_seg";


    bool enable_control = false;
    float control_target_angle = 0.0;
    float control_target_distance = 0.5;
    const char* control_target_angle_param = "control_target_angle";
    const char* control_target_distance_param = "control_target_distance";
    const char* enable_control_param = "enable_control";

    float control_interpolate_dist = 0.05;
    const char* control_interpolate_dist_param = "control_interpolate_dist";

    // if angle_diff is bigger than rotate_angle_up_bound
    // robot should rotate to minimize angle_diff first
    // then move forward
    float rotate_angle_up_bound = 0.3;

    float control_dist_tolerance = 0.01;
    float control_angle_tolerance = 0.01;

    float control_forward_vel_tolerance = 0.01;
    float control_rotate_vel_tolerance = 0.01;

    const char* control_dist_tolerance_param = "control_dist_tolerance";
    const char* control_angle_tolerance_param = "control_angle_tolerance";
    const char* control_forward_vel_tolerance_param = "control_forward_vel_tolerance";
    const char* control_rotate_vel_tolerance_param = "control_rotate_vel_tolerance";


    float rotate_pid_control_kp = 0.1;
    float rotate_pid_control_ki = 0.0;
    float rotate_pid_control_kd = 0.0;
    float rotate_pid_control_scale = 5.0;

    const char* rotate_pid_control_kp_param = "rotate_pid_control_kp";
    const char* rotate_pid_control_ki_param = "rotate_pid_control_ki";
    const char* rotate_pid_control_kd_param = "rotate_pid_control_kd";
    const char* rotate_pid_control_scale_param = "rotate_pid_control_scale";

    float forward_pid_control_kp = 0.1;
    float forward_pid_control_ki = 0.0;
    float forward_pid_control_kd = 0.0;
    float forward_pid_control_scale = 0.0;

    const char* forward_pid_control_kp_param = "forward_pid_control_kp";
    const char* forward_pid_control_ki_param = "forward_pid_control_ki";
    const char* forward_pid_control_kd_param = "forward_pid_control_kd";
    const char* forward_pid_control_scale_param = "forward_pid_control_scale";


    float static_rotate_vel = 0.1;
    float static_forward_vel = 0.1;
    const char* static_rotate_vel_param = "static_rotate_vel";
    const char* static_forward_vel_param = "static_forward_vel";


    float final_control_dist = 0.05;
    const char* final_control_dist_param = "final_control_dist";

    float final_control_angle = 0.05;
    const char* final_control_angle_param = "final_control_angle";



    // control command
    const char* status_param = "status";
    const char* run_param = "run";
    int run_command = 0;
    int run_command_last = 0;
    int start_run = 0;
    int control_finished_cnt = 0;

    //tf
    std::string base_frame = "base_link";
    std::string odom_frame = "odom";
    std::string laser_frame = "base_laser";
    std::string map_frame = "map";
    tf::TransformListener tl_;
    transform::Transform2d tf_base_laser;
    transform::Transform2d tf_base_laser_inv;

    transform::Transform2d tf_odom_base;
    tf::StampedTransform transform;
    bool tf_get_base_laser = false;
    bool tf_get_odom_base = false;

    // laser points

    std::vector<geometry::Point> filter_points_laser(100);
    std::vector<geometry::Point> filter_points_base(100);
    std::vector<std::vector<geometry::Point>> circle_filter_points_segments;
    std::vector<std::vector<std::vector<geometry::Point>>> shelf_filter_points_segments;


    auto compute_leg_pose = [&]{

        if(std::strcmp(mode.c_str(), MODE_SHELF4) == 0){

#if 0
            float leg1[2] = {0.5*shelf_len_y_x[1],0.5*shelf_len_y_x[0]};
            float leg2[2] = {0.5*shelf_len_y_x[1],-0.5*shelf_len_y_x[0]};
            float leg3[2] = {-0.5*shelf_len_y_x[1],0.5*shelf_len_y_x[0]};
            float leg4[2] = {-0.5*shelf_len_y_x[1],-0.5*shelf_len_y_x[0]};

#endif
            float leg1[2] = {0.0, 0.5*shelf_len_y_x[0]};
            float leg2[2] = {0.0, -0.5*shelf_len_y_x[0]};
            float leg3[2] = {-shelf_len_y_x[1],0.5*shelf_len_y_x[0]};
            float leg4[2] = {-shelf_len_y_x[1],-0.5*shelf_len_y_x[0]};


            shelf_leg_base_pose.resize(4);

            detect_target_relative_tf.mul(leg1, 1, shelf_leg_base_pose[0].pose_in_base);
            detect_target_relative_tf.mul(leg2, 1, shelf_leg_base_pose[1].pose_in_base);
            detect_target_relative_tf.mul(leg3, 1, shelf_leg_base_pose[2].pose_in_base);
            detect_target_relative_tf.mul(leg4, 1, shelf_leg_base_pose[3].pose_in_base);

//            shelf_empty_region
            shelf_filter_points_segments.resize(4);
            shelf_leg_front_pose.resize(4);
            shelf_leg_min_matched_num = 4;
        }else if(std::strcmp(mode.c_str(), MODE_SHELF2) == 0){
            float leg1[2] = {0.0,0.5*shelf_len_y_x[0]};
            float leg2[2] = {0.0,-0.5*shelf_len_y_x[0]};

            shelf_leg_base_pose.resize(2);

            detect_target_relative_tf.mul(leg1, 1, shelf_leg_base_pose[0].pose_in_base);
            detect_target_relative_tf.mul(leg2, 1, shelf_leg_base_pose[1].pose_in_base);
            shelf_filter_points_segments.resize(2);
            shelf_leg_front_pose.resize(2);
            shelf_leg_min_matched_num= 2;
        }
    };

    auto load_params = [&]{
        nh_private.getParam(mode_param, mode);

        nh_private.getParam(detect_target_relative_pose_param, detect_target_relative_pose);

        nh_private.getParam(shelf_len_y_x_param, shelf_len_y_x);
        shelf_len_y_x_real = shelf_len_y_x;

        if(detect_target_relative_pose.size() != 5 ){
            std::cerr << "detect_target_relative_pose size error"<< std::endl;
            return -1;
        }


        std::cout << "load detect_target_relative_pose : " << detect_target_relative_pose[0] << ", " << detect_target_relative_pose[1] << ", " << detect_target_relative_pose[2] << ", " << detect_target_relative_pose[3] << std::endl;
        if(std::abs(detect_target_relative_pose[0]) < 0.01){
            std::cerr << "detect_target_relative_pose value error"<< std::endl;

            return -1;
        }
        detect_target_relative_tf.set(detect_target_relative_pose[0],detect_target_relative_pose[1],detect_target_relative_pose[2]);



        if(std::strcmp(mode.c_str(), MODE_SHELF4) == 0){

            if(shelf_len_y_x.size() != 2 ){
                std::cerr << "shelf_len_y_x size error"<< std::endl;

                return -1;
            }

            if(shelf_len_y_x[0] < 0.1 || shelf_len_y_x[1] < 0.1){
                std::cerr << "shelf_len_y_x value error"<< std::endl;
                return -1;
            }

            // compute four legs
            compute_leg_pose();

        }

        if(std::strcmp(mode.c_str(), MODE_SHELF2) == 0){

            if(shelf_len_y_x.size() <= 1 ){
                std::cerr << "shelf_len_y_x size error"<< std::endl;

                return -1;
            }

            if(shelf_len_y_x[0] < 0.1 ){
                std::cerr << "shelf_len_y_x value error"<< std::endl;
                return -1;
            }

            // compute two legs
            compute_leg_pose();
        }



        nh_private.getParam(front_radius_param, front_radius);

        nh_private.getParam(front_min_num_param, front_min_num);


        nh_private.getParam(pattern_match_radius_param, pattern_match_radius);

        nh_private.getParam(shelf_rect_diff_param, shelf_rect_diff);



        nh_private.getParam(detect_target_stop_update_dist_param, detect_target_stop_update_dist);


        nh_private.getParam(max_fit_error_param, max_fit_error);
        nh_private.getParam(sleep_time_param, sleep_time);
        nh_private.getParam(tf_wait_time_param, tf_wait_time);
        nh_private.getParam(range_max_param, range_max);
        nh_private.getParam(range_min_param, range_min);
        nh_private.getParam(scan_point_jump_param, scan_point_jump);
        nh_private.getParam(scan_noise_angle_param, scan_noise_angle);


        nh_private.getParam(filer_angle_min_param, filer_angle_min);
        nh_private.getParam(filer_angle_max_param, filer_angle_max);

        nh_private.getParam(box_filter_min_x_param, box_filter_min_x);
        nh_private.getParam(box_filter_max_x_param, box_filter_max_x);
        nh_private.getParam(box_filter_min_y_param, box_filter_min_y);
        nh_private.getParam(box_filter_max_y_param, box_filter_max_y);

        nh_private.getParam(circle_radius_param, circle_radius);
        nh_private.getParam(circle_edge_radius_param, circle_edge_radius);
        nh_private.getParam(circle_min_num_in_radius_param, circle_min_num_in_radius);

        nh_private.getParam(circle_edge_range_offset_param, circle_edge_range_offset);


        nh_private.getParam(circle_min_point_num_param, circle_min_point_num);


        nh_private.getParam(split_dist_x_param, split_dist_x);
        nh_private.getParam(split_dist_y_param, split_dist_y);
        nh_private.getParam(split_dist_r_param, split_dist_r);

        nh_private.getParam(min_point_num_in_seg_param, min_point_num_in_seg);

        nh_private.getParam(control_target_angle_param, control_target_angle);
        nh_private.getParam(control_target_distance_param, control_target_distance);
        nh_private.getParam(enable_control_param, enable_control);

        nh_private.getParam(control_interpolate_dist_param, control_interpolate_dist);


        nh_private.getParam(control_dist_tolerance_param, control_dist_tolerance);
        nh_private.getParam(control_angle_tolerance_param, control_angle_tolerance);
        nh_private.getParam(control_forward_vel_tolerance_param, control_forward_vel_tolerance);
        nh_private.getParam(control_rotate_vel_tolerance_param,control_rotate_vel_tolerance);


        nh_private.getParam(static_rotate_vel_param, static_rotate_vel);
        nh_private.getParam(static_forward_vel_param, static_forward_vel);



        nh_private.getParam(rotate_pid_control_kp_param, rotate_pid_control_kp);
        nh_private.getParam(rotate_pid_control_ki_param, rotate_pid_control_ki);
        nh_private.getParam(rotate_pid_control_kd_param, rotate_pid_control_kd);
        nh_private.getParam(forward_pid_control_kp_param, forward_pid_control_kp);
        nh_private.getParam(forward_pid_control_ki_param, forward_pid_control_ki);
        nh_private.getParam(forward_pid_control_kd_param, forward_pid_control_kd);


        nh_private.getParam(final_control_dist_param, final_control_dist);
        nh_private.getParam(final_control_angle_param, final_control_angle);


        return 1;
    };

    start_run = load_params();

    common::Suspend suspend;
    sensor::ScanToPoints scan_handler;

    scan_handler.scan_max_jump = scan_point_jump;
    scan_handler.scan_noise_angle = scan_noise_angle;

    bool scan_get_data = false;
    ros::Time scan_time;
    auto laser_cb = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        laser_frame.assign(msg->header.frame_id);
        scan_time = msg->header.stamp;
        scan_get_data = true;
        scan_handler.getLocalPoints(msg->ranges, msg->angle_min, msg->angle_increment, range_min, range_max,filer_angle_min,filer_angle_max);
        scan_get_data = scan_handler.range_valid_num > 10;
    };
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laser_cb);


    nav_msgs::Odometry robot_odom;
    bool odom_get_data = false;
    auto odom_cb = [&](const  nav_msgs::OdometryConstPtr &msg){
        robot_odom = *msg;
        odom_get_data = true;
    };
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odom_cb);



    const char* cmd_vel_topic = "/cmd_vel";
    const char* cloud_target_topic = "detect_target";
    const char* debug_info_topic = "debug_info";
    const char* cloud_filtered_topic = "cloud_filtered";
    const char* detect_results_topic = "detect_results";

    ros::Publisher cmd_vel_pub =  nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

    geometry_msgs::Twist cmd_vel_msg;

    ros::Publisher cloud_target_pub = nh_private.advertise<visualization_msgs::MarkerArray>(cloud_target_topic,1);

    ros::Publisher debug_info_pub = nh_private.advertise<geometry_msgs::Point>(debug_info_topic,1);
    geometry_msgs::Point debug_info_msg ;

    ros::Publisher cloud_filtered_pub = nh_private.advertise<sensor_msgs::PointCloud2>(cloud_filtered_topic, 1);
    sensor_msgs::PointCloud2 cloud_filtered_msg;

    ros::Publisher detect_results_pub = nh_private.advertise<geometry_msgs::PoseArray>(detect_results_topic, 1);
    geometry_msgs::PoseArray detect_results_msg;
    geometry_msgs::Pose target_pose;


    detect_results_msg.header.frame_id = "base_link";

    cloud_filtered_msg.header.frame_id = "map";

    sensor::createPointCloud2(cloud_filtered_msg, {"x", "y", "z"});

    std::vector<geometry::Point> cloud_filtered_points;


    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker  marker_msg;
    marker_msg.header.frame_id = "base_link";//fixed_frame;
    marker_msg.type = visualization_msgs::Marker::CYLINDER;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 2*circle_radius;
    marker_msg.scale.y = 2*circle_radius;
    marker_msg.scale.z = 2*circle_radius;
    marker_msg.header.stamp = ros::Time();
    marker_msg.color.a = 0.5; // Don't forget to set the alpha!
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_array_msg.markers.resize(1,marker_msg);




    std::vector<float> points_in_base(500);

    int filter_point_num = 0;


    float pid_dt = 0.001*sleep_time;


    PID rotate_pid(pid_dt, 0.1, -0.1, rotate_pid_control_kp, rotate_pid_control_kd, rotate_pid_control_ki);
    common::Time rotate_pid_t1 = common::FromUnixNow();

    PID forward_pid(pid_dt, 0.1, -0.1, forward_pid_control_kp, forward_pid_control_kd, forward_pid_control_ki);



    bool first_detect_ok = false;

    auto pending_task = [&]{
        run_command_last = run_command;
        suspend.sleep(1000.0);
    };
    auto restart_task = [&]{
        run_command_last = run_command;
        first_detect_ok = false;
        detect_target_absolute_pose_computed = false;
        detect_target_stop_update = false;
        start_run = load_params();
        control_finished_cnt = 0;
        rotate_pid.reset();
        forward_pid.reset();

    };

    while (ros::ok()){
        nh_private.getParam(run_param,run_command);

        if(run_command == 0){
            pending_task();
            continue;
        }
        if(run_command && !run_command_last){
            restart_task();
        }

        if(start_run == -1 || start_run == 0){
            pending_task();
            continue;
        }


        ros::spinOnce();

        if(scan_get_data){

            // transform points to base frame
            // apply box filter
            //

            if (!tf_get_base_laser && scan_get_data) {
                try {
                    tl_.lookupTransform(base_frame, laser_frame, ros::Time(0), transform);
                    tf_get_base_laser = true;
                    tf_base_laser.set(transform.getOrigin().x(), transform.getOrigin().y(),
                                      tf::getYaw(transform.getRotation()));

                    tf_base_laser_inv = tf_base_laser.inverse();
                } catch (tf::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                    continue;
                }
            }


            tf_get_odom_base = false;
            // lookup odom base_link
            try {
                tl_.waitForTransform(odom_frame, base_frame, scan_time, ros::Duration(tf_wait_time));

                tl_.lookupTransform(odom_frame, base_frame, scan_time, transform);
//                tf_get_odom_base = true;
                tf_odom_base.set(transform.getOrigin().x(), transform.getOrigin().y(),
                                 tf::getYaw(transform.getRotation()));
                tf_get_odom_base = true;

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }

            PLOGD << "tf_base_laser: " <<  tf_base_laser << std::endl;

            tf_base_laser.mul(scan_handler.local_xy_points, scan_handler.range_valid_num, points_in_base);

            PLOGD << "range_valid_num: " <<  scan_handler.range_valid_num << std::endl;
#if 0
            for(int i = 0; i < scan_handler.range_valid_num;i++){
                std::cout << "== "<< i << ", " << scan_handler.local_xy_points[i+i] << ", " << scan_handler.local_xy_points[i+i+1]  <<"\n";

            }
#endif


            cloud_filtered_points.clear();

            if(std::strcmp(mode.c_str(), MODE_CIRCLE) == 0){

//                LaserScanBoxFilter(scan_handler.local_xy_points, points_in_base, scan_handler.range_valid_num, filter_points_laser,filter_points_base, filter_point_num,box_filter_min_x,box_filter_max_x,box_filter_min_y,box_filter_max_y );

                LaserScanRadiusFilter(scan_handler.local_xy_points, points_in_base, scan_handler.range_valid_num, filter_points_laser,filter_points_base, filter_point_num, detect_target_relative_pose[0],detect_target_relative_pose[1],

                                      detect_target_absolute_pose_computed? detect_target_relative_pose[4] : detect_target_relative_pose[3] );


                PLOGD << "filter_point_num: " << filter_point_num << std::endl;
#if 0
                for(int i = 0; i < filter_point_num;i++){
                std::cout << "** "<< i << ", " << filter_points_laser[i].x << ", " << filter_points_laser[i].y  <<"\n";

            }
#endif

                LaserScanSegment(filter_points_laser,circle_filter_points_segments, filter_point_num,split_dist_x, split_dist_y, split_dist_r, min_point_num_in_seg);


                std::vector<geometry::DetectTarget> find_result(circle_filter_points_segments.size());
                for(int i = 0 ; i <circle_filter_points_segments.size(); i++ ){
                    FindCircle(circle_filter_points_segments[i], circle_radius,circle_edge_radius,circle_edge_range_offset, circle_min_point_num ,find_result[i].pose_in_laser[0],find_result[i].pose_in_laser[1],find_result[i].match_error);
                    tf_base_laser.mul(find_result[i].pose_in_laser, 1, find_result[i].pose_in_base);

                }





                int best_result_id = 0;
                if(find_result.empty()){
                    best_result_id = -1;
                }else{
                    std::sort(find_result.begin(),find_result.end(),[](auto& v1,auto& v2){
                        return (std::abs(v1.pose_in_base[0])+std::abs(v1.pose_in_base[1])) < ( std::abs(v2.pose_in_base[0]) + std::abs(v2.pose_in_base[1]));
                    });
                    while(find_result[best_result_id].match_error > max_fit_error){

                        best_result_id++;
                        if(best_result_id == find_result.size()){
                            best_result_id = -1;
                            break;
                        }
                    }
                }


                if(best_result_id == -1){

                    if(!detect_target_absolute_pose_computed){
                        PLOGD << "find circle fail " << std::endl;
                        continue;
                    }else{
                        if(tf_get_odom_base){


                            detect_target_relative_tf = tf_odom_base.inverse()*detect_target_absolute_tf;

                        }

                    }
                }else{


                    geometry::DetectTarget best_result = find_result[best_result_id];

                    detect_target_relative_tf.set(best_result.pose_in_base[0] , best_result.pose_in_base[1], std::atan2(best_result.pose_in_base[1],best_result.pose_in_base[0]));
                    if(tf_get_odom_base){

                        detect_target_absolute_pose_computed = true;

                        detect_target_absolute_tf = tf_odom_base * detect_target_relative_tf;

                    }


                }


                first_detect_ok = true;

                PLOGD << "control_target_angle: " << control_target_angle << std::endl;
                PLOGD << "control_target_distance: " << control_target_distance << std::endl;
                PLOGD << "target pose in base ,  " << detect_target_relative_tf.x() << ", " << detect_target_relative_tf.y() << std::endl;

                detect_target_relative_pose[0] = detect_target_relative_tf.x();
                detect_target_relative_pose[1] = detect_target_relative_tf.y();


                marker_array_msg.markers[0].header.stamp = scan_time;
//                marker_array_msg.markers[0].header.frame_id.assign(laser_frame);
                marker_array_msg.markers[0].pose.position.x =detect_target_relative_tf.x();
                marker_array_msg.markers[0].pose.position.y =detect_target_relative_tf.y();
                marker_array_msg.markers[0].id = 1;

                cloud_target_pub.publish(marker_array_msg);



                float marker_angle = detect_target_relative_tf.yaw();

                marker_angle = std::abs(marker_angle - control_target_angle) < M_PI ? (marker_angle) : ( marker_angle + ((marker_angle- control_target_angle) > 0.0 ? -M_PI*2: M_PI*2) );

                float angle_diff = marker_angle - control_target_angle;

                float marker_distance = std::sqrt(detect_target_relative_tf.x()*detect_target_relative_tf.x() + detect_target_relative_tf.y()*detect_target_relative_tf.y() );
                float distance_diff = marker_distance - control_target_distance;

                PLOGD << "angle_diff: " << angle_diff << std::endl;
                PLOGD << "distance_diff: " << distance_diff << std::endl;

                {
                    if( std::abs(angle_diff) > final_control_angle){
                        cmd_vel_msg.angular.z =  final_control_angle*rotate_pid_control_kp*(angle_diff> 0.0 ? 1.0:-1.0);;

                    }else{
                        double rotate_pid_inc = rotate_pid.calculate(angle_diff, 0.0);
                        PLOGD << "rotate_pid_inc: " << rotate_pid_inc << std::endl;

                        if(std::abs(rotate_pid_inc) < control_rotate_vel_tolerance || std::abs(angle_diff) < control_angle_tolerance){
                            rotate_pid_inc = 0.0;
                        }
                        cmd_vel_msg.angular.z =  rotate_pid_inc;

                    }
                }





                float target_pose_x = distance_diff * std::cos(marker_angle);


//            if(std::abs(angle_diff) < 0.02)
                {

                    if(std::abs(target_pose_x) > final_control_dist){
                        cmd_vel_msg.linear.x =  final_control_dist*forward_pid_control_kp*(target_pose_x> 0.0 ? 1.0:-1.0);
                    }else{
                        double forward_pid_inc = forward_pid.calculate(target_pose_x, 0.0);

                        PLOGD << "forward_pid_inc: " << forward_pid_inc << std::endl;
                        if(std::abs(forward_pid_inc) < control_forward_vel_tolerance || std::abs(target_pose_x) < control_dist_tolerance){
                            forward_pid_inc = 0.0;
                        }
                        cmd_vel_msg.linear.x =  forward_pid_inc;
                    }

                    debug_info_msg.x =target_pose_x;
                    debug_info_msg.y =marker_angle;

                    debug_info_pub.publish(debug_info_msg);

                }


                if(!enable_control){
                    continue;
                }
                PLOGD << "cmd_vel_msg.angular.z : " <<   cmd_vel_msg.angular.z  << std::endl;
                PLOGD << "cmd_vel_msg.linear.x : " <<  cmd_vel_msg.linear.x << std::endl;
                cmd_vel_pub.publish(cmd_vel_msg);

                if(std::abs(target_pose_x)<control_dist_tolerance && std::abs(angle_diff) < control_angle_tolerance){
                    PLOGD << "control finished test : " <<  control_finished_cnt << std::endl;

                    control_finished_cnt++;
                    if(control_finished_cnt > 10){
                        start_run = 0;
                    }
                }
            }else if(std::strcmp(mode.c_str(), MODE_SHELF4) == 0){

//                compute_leg_pose();
                // filter and segment
                // each leg has a clusters of points : std::vector<std::vector<Point>>

                marker_array_msg.markers.resize(shelf_leg_base_pose.size(),marker_msg);
                shelf_leg_front_pose.resize(shelf_leg_base_pose.size());

                detect_results_msg.poses.clear();
                shelf_filter_points_segments.resize(shelf_leg_base_pose.size());

                for(int i =0 ; i < shelf_leg_base_pose.size(); i++){


                    PLOGD << "check leg: " << i << ", " << shelf_leg_base_pose[i].pose_in_base[0] << ", " << shelf_leg_base_pose[i].pose_in_base[1] << std::endl;

                    tf_base_laser_inv.mul(shelf_leg_base_pose[i].pose_in_base, 1, shelf_leg_base_pose[i].pose_in_laser);


                    LaserScanRadiusFilter(scan_handler.local_xy_points, points_in_base, scan_handler.range_valid_num, filter_points_laser,filter_points_base, filter_point_num,
                                          shelf_leg_base_pose[i].pose_in_base[0],shelf_leg_base_pose[i].pose_in_base[1],

                                          detect_target_absolute_pose_computed? detect_target_relative_pose[4] : detect_target_relative_pose[3] );
                    PLOGD << "check filter_points_laser size : " << filter_points_laser.size() << std::endl;
                    PLOGD << "check filter_point_num : " << filter_point_num << std::endl;

                    LaserScanSegment(filter_points_laser,shelf_filter_points_segments[i], filter_point_num,split_dist_x, split_dist_y, split_dist_r, min_point_num_in_seg);



                    shelf_leg_front_pose[i].resize(shelf_filter_points_segments[i].size());
                    for(int j = 0 ; j < shelf_filter_points_segments[i].size();j++){

                        perception::FindFrontEdge(shelf_filter_points_segments[i][j],front_min_num, front_radius ,shelf_leg_front_pose[i][j].pose_in_laser[0], shelf_leg_front_pose[i][j].pose_in_laser[1],shelf_leg_front_pose[i][j].match_error );

                        if(shelf_leg_front_pose[i][j].match_error < 0.05){
                            tf_base_laser.mul(shelf_leg_front_pose[i][j].pose_in_laser, 1, shelf_leg_front_pose[i][j].pose_in_base);

                            PLOGD << "check FindFrontEdge pose_in_laser: " << shelf_leg_front_pose[i][j].pose_in_laser[0] << ", " << shelf_leg_front_pose[i][j].pose_in_laser[1]  << std::endl;
                            PLOGD << "check FindFrontEdge pose_in_base : " << shelf_leg_front_pose[i][j].pose_in_base[0] << ", " << shelf_leg_front_pose[i][j].pose_in_base[1]  << std::endl;
                            target_pose.position.x = shelf_leg_front_pose[i][j].pose_in_laser[0];
                            target_pose.position.y = shelf_leg_front_pose[i][j].pose_in_laser[1];

                            math::yaw_to_quaternion(std::atan2(shelf_leg_front_pose[i][j].pose_in_laser[1],shelf_leg_front_pose[i][j].pose_in_laser[0]),
                                                    target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);

                            detect_results_msg.poses.emplace_back(target_pose);

                        }

                        cloud_filtered_points.insert(cloud_filtered_points.end(),shelf_filter_points_segments[i][j].begin(),shelf_filter_points_segments[i][j].end());

                    }


//                    perception::FindMatchPattern()

                    marker_array_msg.markers[i].header.stamp = scan_time;
//                    marker_array_msg.markers[i].scale.x = 2.0*(detect_target_absolute_pose_computed? detect_target_relative_pose[4] : detect_target_relative_pose[3] );
//                    marker_array_msg.markers[i].scale.y = marker_array_msg.markers[i].scale.x;

//                    marker_array_msg.markers[i].header.frame_id.assign(laser_frame);
                    marker_array_msg.markers[i].pose.position.x = shelf_leg_base_pose[i].pose_in_base[0];
                    marker_array_msg.markers[i].pose.position.y = shelf_leg_base_pose[i].pose_in_base[1];
                    marker_array_msg.markers[i].id = i;

                }



                /*
                 1. cluster segment result save in  shelf_filter_points_segments
                 shelf_filter_points_segments : [ L1:[c1:[p1,p2,p3],c2,c3] ,L2:[c1,c2,c3],L3:[],L4:[]  ]


                 2. for each cluster
                 compute front end pose in laser frame
                 check L1 L2 with leg distance
                 only at first time, check L3 L4 with leg distance, if L3 L4 exist

                 compute L1 L2 direction center

                 */


                perception::FindMatchPattern(shelf_leg_base_pose,shelf_leg_front_pose,pattern_match_radius, shelf_leg_min_matched_num,shelf_leg_match_result );
                PLOGD << "shelf_leg_match_result.size " << shelf_leg_match_result.size() << std::endl;

                int best_result_id = 0;
                if(shelf_leg_match_result.empty()){
                    best_result_id = -1;
                }else{
                    std::sort(shelf_leg_match_result.begin(),shelf_leg_match_result.end(), [](auto& v1, auto& v2){

                        return v1.back() < v2.back();
                    });


                    float best_edge_12 = 1000.0;
                    int best_edge_id = -1;
                    //
                    if(!first_detect_ok){


                        while(best_result_id < shelf_leg_match_result.size()){


                            /*

                             l1     l2


                             l3     l4

                                 |x
                               y-| robot

                             *
                             * */
                            auto& result = shelf_leg_match_result[best_result_id];

                            PLOGD << "check match result: " << result[0] << ", " << result[1] << ", " << result[2] << ", " << result[3] << std::endl;
                            auto& detect_leg1 = shelf_leg_front_pose[0][result[0]];
                            auto& detect_leg2 = shelf_leg_front_pose[1][result[1]];

                            auto& detect_leg3 = shelf_leg_front_pose[2][result[2]];
                            auto& detect_leg4 = shelf_leg_front_pose[3][result[3]];

                            float edge_12 = std::sqrt( (detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );
                            float edge_34 = std::sqrt( (detect_leg4.pose_in_laser[0] - detect_leg3.pose_in_laser[0])*(detect_leg4.pose_in_laser[0] - detect_leg3.pose_in_laser[0]) +  (detect_leg4.pose_in_laser[1] - detect_leg3.pose_in_laser[1])*(detect_leg4.pose_in_laser[1] - detect_leg3.pose_in_laser[1])   );

                            float edge_13 = std::sqrt( (detect_leg3.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg3.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg3.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg3.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );
                            float edge_24 = std::sqrt( (detect_leg4.pose_in_laser[0] - detect_leg2.pose_in_laser[0])*(detect_leg4.pose_in_laser[0] - detect_leg2.pose_in_laser[0]) +  (detect_leg4.pose_in_laser[1] - detect_leg2.pose_in_laser[1])*(detect_leg4.pose_in_laser[1] - detect_leg2.pose_in_laser[1])   );



                            float angle31 = std::atan2(detect_leg1.pose_in_laser[1] - detect_leg3.pose_in_laser[1]  ,detect_leg1.pose_in_laser[0] - detect_leg3.pose_in_laser[0]);
                            float angle42 = std::atan2(detect_leg2.pose_in_laser[1] - detect_leg4.pose_in_laser[1]  ,detect_leg2.pose_in_laser[0] - detect_leg4.pose_in_laser[0]);
                            float angle21 = std::atan2(detect_leg1.pose_in_laser[1] - detect_leg2.pose_in_laser[1]  ,detect_leg1.pose_in_laser[0] - detect_leg2.pose_in_laser[0]);
                            float angle43 = std::atan2(detect_leg3.pose_in_laser[1] - detect_leg4.pose_in_laser[1]  ,detect_leg3.pose_in_laser[0] - detect_leg4.pose_in_laser[0]);

                            float rect_angle1 = angle21 - angle_normalise(angle31, angle21);
                            float rect_angle2 = angle21 - angle_normalise(angle42, angle21);
                            float rect_angle3 = angle43 - angle_normalise(angle31, angle43);
                            float rect_angle4 = angle43 - angle_normalise(angle42, angle43);

                            char msg[500];
                            sprintf(msg,"shelf_len_y_x_real: [%.3f,%.3f], [%.3f,%.3f], angle31:%.3f, angle42:%.3f, angle21:%.3f, angle43:%.3f, rect_angle1:%.3f, rect_angle2:%.3f, rect_angle3:%.3f, rect_angle4:%.3f",
                                    edge_12, edge_13 , edge_34 ,edge_24,angle31,angle42 ,angle21,angle43, rect_angle1, rect_angle2, rect_angle3, rect_angle4);
                            PLOGD << msg << std::endl;

                            bool valid = std::abs(edge_12 - edge_34) <  shelf_rect_diff[0]
                                    && std::abs(edge_13 - edge_24) <  shelf_rect_diff[1]
                                    && std::abs(rect_angle1 - M_PI_2) < shelf_rect_diff[2]
                                    && std::abs(rect_angle2 - M_PI_2) < shelf_rect_diff[2]
                                    && std::abs(rect_angle3 - M_PI_2) < shelf_rect_diff[2]
                                    && std::abs(rect_angle4 - M_PI_2) < shelf_rect_diff[2]
//                                    && std::abs(detect_leg1.pose_in_laser[0]) > std::abs(detect_leg3.pose_in_laser[0])
//                                    && std::abs(detect_leg2.pose_in_laser[0]) > std::abs(detect_leg4.pose_in_laser[0])

                                    ;



                            result.back() = 500.0f*(std::abs(edge_12 - edge_34) + std::abs(edge_13 - edge_24));
                            if(valid){

                                if(edge_12 < best_edge_12){

                                    best_edge_12 = edge_12;
                                    best_edge_id = best_result_id;

                                    PLOGD << "get valid rect, write to file" << std::endl;

                                    shelf_len_y_x_real[0] = edge_12;
                                    shelf_len_y_x_real[1] = edge_34;
                                    shelf_leg_base_pose.resize(2);
                                    shelf_leg_min_matched_num = 2;


                                    nh_private.setParam(shelf_len_y_x_real_param, shelf_len_y_x_real);
                                    std::ofstream t("/tmp/shelf.txt");
                                    t << msg;
                                }
                            }

                            best_result_id++;
                            if(best_result_id == shelf_leg_match_result.size()){
                                best_result_id = -1;
                                break;
                            }
                        }

                        best_result_id = best_edge_id;



                    }else{

                        while(best_result_id < shelf_leg_match_result.size()){

                            auto& result = shelf_leg_match_result[best_result_id];

                            PLOGD << "check match result: " << result[0] << ", " << result[1] << ", " << result[2] << ", " << result[3] << std::endl;
                            auto& detect_leg1 = shelf_leg_front_pose[0][result[0]];
                            auto& detect_leg2 = shelf_leg_front_pose[1][result[1]];
                            float edge_12 = std::sqrt( (detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );

                            if(edge_12 < best_edge_12){

                                best_edge_12 = edge_12;
                                best_edge_id = best_result_id;

                            }

                            best_result_id++;
                            if(best_result_id == shelf_leg_match_result.size()){
                                best_result_id = -1;
                                break;
                            }

                        }
                        best_result_id = best_edge_id;


                    }
                }


                cloud_target_pub.publish(marker_array_msg);


                if(detect_target_absolute_pose_computed && detect_target_stop_update){
                    best_result_id = -1;
                }

                if(best_result_id == -1){

                    if(!detect_target_absolute_pose_computed){
                        PLOGD << "find target fail " << std::endl;

                        detect_results_msg.header.stamp = scan_time;
                        detect_results_msg.header.frame_id.assign(laser_frame);

                        detect_results_pub.publish(detect_results_msg);
                        continue;
                    }else{
                        if(tf_get_odom_base){
                            detect_target_relative_tf = tf_odom_base.inverse()*detect_target_absolute_tf;

                        }

                    }
                }else{


                    // check matched legs
                    auto& best_result = shelf_leg_match_result[best_result_id];

                    auto& detect_leg1 = shelf_leg_front_pose[0][best_result[0]];
                    auto& detect_leg2 = shelf_leg_front_pose[1][best_result[1]];
                    float edge_12 = std::sqrt( (detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );
                    shelf_len_y_x_real[0] = edge_12;


                    PLOGD << "check two leg" << std::endl;
                    PLOGD << shelf_leg_base_pose[0].pose_in_base[0] << ", " << shelf_leg_base_pose[0].pose_in_base[1] << ", " << shelf_leg_base_pose[0].pose_in_laser[0] << ", " << shelf_leg_base_pose[0].pose_in_laser[1] << std::endl;
                    PLOGD << shelf_leg_base_pose[1].pose_in_base[0] << ", " << shelf_leg_base_pose[1].pose_in_base[1] << ", " << shelf_leg_base_pose[1].pose_in_laser[0] << ", " << shelf_leg_base_pose[1].pose_in_laser[1] << std::endl;

                    PLOGD << detect_leg1.pose_in_base[0] << ", " << detect_leg1.pose_in_base[1] << ", " << detect_leg1.pose_in_laser[0] << ", " << detect_leg1.pose_in_laser[1] << std::endl;
                    PLOGD << detect_leg2.pose_in_base[0] << ", " << detect_leg2.pose_in_base[1] << ", " << detect_leg2.pose_in_laser[0] << ", " << detect_leg2.pose_in_laser[1] << std::endl;



                    target_pose.position.x = 0.5*(detect_leg1.pose_in_laser[0] + detect_leg2.pose_in_laser[0]);
                    target_pose.position.y = 0.5*(detect_leg1.pose_in_laser[1] + detect_leg2.pose_in_laser[1]);

                    math::yaw_to_quaternion(std::atan2(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1]  ,detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) + M_PI_2,
                                            target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);

                    detect_results_msg.poses.emplace_back(target_pose);


                    detect_target_relative_tf.set(0.5*(detect_leg1.pose_in_base[0] + detect_leg2.pose_in_base[0]),
                                                  0.5*(detect_leg1.pose_in_base[1] + detect_leg2.pose_in_base[1]),
                                                  std::atan2(detect_leg2.pose_in_base[1] - detect_leg1.pose_in_base[1]  ,detect_leg2.pose_in_base[0] - detect_leg1.pose_in_base[0]) + M_PI_2
                                                  );


                    if(tf_get_odom_base){
                        detect_target_absolute_pose_computed = true;
                        detect_target_absolute_tf = tf_odom_base * detect_target_relative_tf;

                    }
                }

                first_detect_ok = true;



                PLOGD << "control_target_angle: " << control_target_angle << std::endl;
                PLOGD << "control_target_distance: " << control_target_distance << std::endl;


                float leg1[2] = {0.0, 0.5*shelf_len_y_x_real[0]};
                float leg2[2] = {0.0, -0.5*shelf_len_y_x_real[0]};
                float control_target_in_shelf[2] = {-control_target_distance,0.0};
                float control_target_in_base[2] = {0.0,0.0};
                float control_interpolate_target_in_base[2] = {0.0,0.0};

                detect_target_relative_tf.mul(leg1, 1, shelf_leg_base_pose[0].pose_in_base);
                detect_target_relative_tf.mul(leg2, 1, shelf_leg_base_pose[1].pose_in_base);

                detect_target_relative_tf.mul(control_target_in_shelf, 1, control_target_in_base);
                PLOGD << "control_target_in_shelf: " << control_target_in_shelf[0]<<", " << control_target_in_shelf[1] << std::endl;
                PLOGD << "control_target_in_base: " << control_target_in_base[0]<<", " << control_target_in_base[1] << std::endl;

                float marker_distance = std::sqrt(control_target_in_base[0]*control_target_in_base[0] + control_target_in_base[1]*control_target_in_base[1]   );

                float interpolate_distance =  control_target_distance + ( (marker_distance> control_interpolate_dist)?(marker_distance -control_interpolate_dist):(0.0)   );
                control_target_in_shelf[0] = -interpolate_distance;
                detect_target_relative_tf.mul(control_target_in_shelf, 1, control_interpolate_target_in_base);
                PLOGD << "interpolate control_interpolate_target_in_base: " << control_interpolate_target_in_base[0]<<", " << control_interpolate_target_in_base[1] << std::endl;


                sensor::LaserScanToPointCloud2(cloud_filtered_points,cloud_filtered_points.size(),cloud_filtered_msg);
                cloud_filtered_msg.header.stamp = scan_time;
                cloud_filtered_msg.header.frame_id.assign(laser_frame);

                cloud_filtered_pub.publish(cloud_filtered_msg);

                detect_results_msg.header.stamp = scan_time;
                detect_results_msg.header.frame_id.assign(laser_frame);

                detect_results_pub.publish(detect_results_msg);



                float marker_angle = std::atan2(control_interpolate_target_in_base[1],control_interpolate_target_in_base[0]);
                float angle_diff = marker_angle - control_target_angle;

                marker_distance = std::sqrt(control_interpolate_target_in_base[0]*control_interpolate_target_in_base[0] + control_interpolate_target_in_base[1]*control_interpolate_target_in_base[1]   );
                float distance_diff = marker_distance ;//- control_target_distance;

                PLOGD << "angle_diff: " << angle_diff << std::endl;
                PLOGD << "distance_diff: " << distance_diff << std::endl;

                PLOGD << "detect_target_stop_update: " << detect_target_stop_update << std::endl;



                {
                    if( std::abs(angle_diff) > final_control_angle){
                        cmd_vel_msg.angular.z =  final_control_angle*rotate_pid_control_kp*(angle_diff> 0.0 ? 1.0:-1.0);;
                        cmd_vel_msg.angular.z =  static_rotate_vel*(angle_diff> 0.0 ? 1.0:-1.0);;

                    }else{
                        double rotate_pid_inc = rotate_pid.calculate(angle_diff, 0.0);
                        PLOGD << "rotate_pid_inc: " << rotate_pid_inc << std::endl;

                        if(std::abs(rotate_pid_inc) < control_rotate_vel_tolerance || std::abs(angle_diff) < control_angle_tolerance){
                            rotate_pid_inc = 0.0;
                        }
                        cmd_vel_msg.angular.z =  rotate_pid_inc;

                    }
                }

                if(std::abs(control_target_in_base[0]) <detect_target_stop_update_dist){
                    cmd_vel_msg.angular.z =  0.0;

                }




                float target_pose_x = distance_diff * std::cos(marker_angle);


//            if(std::abs(angle_diff) < 0.02)
                {

                    if(std::abs(target_pose_x) > final_control_dist){
                        cmd_vel_msg.linear.x =  final_control_dist*forward_pid_control_kp*(target_pose_x> 0.0 ? 1.0:-1.0);
                        cmd_vel_msg.linear.x =  static_forward_vel;

                    }else{
                        double forward_pid_inc = forward_pid.calculate(target_pose_x, 0.0);

                        PLOGD << "forward_pid_inc: " << forward_pid_inc << std::endl;
                        if(std::abs(forward_pid_inc) < control_forward_vel_tolerance || std::abs(target_pose_x) < control_dist_tolerance){
                            forward_pid_inc = 0.0;
                        }
                        cmd_vel_msg.linear.x =  forward_pid_inc;
                    }

                    debug_info_msg.x =target_pose_x;
                    debug_info_msg.y =marker_angle;

                    debug_info_pub.publish(debug_info_msg);

                }



                PLOGD << "cmd_vel_msg.angular.z : " <<   cmd_vel_msg.angular.z  << std::endl;
                PLOGD << "cmd_vel_msg.linear.x : " <<  cmd_vel_msg.linear.x << std::endl;
                if(!enable_control){
                    continue;
                }

                cmd_vel_pub.publish(cmd_vel_msg);

                if(std::abs(target_pose_x)<control_dist_tolerance
//                && std::abs(angle_diff) < control_angle_tolerance
                ){
                    PLOGD << "control finished test : " <<  control_finished_cnt << std::endl;

                    control_finished_cnt++;
                    if(control_finished_cnt > 10){
                        start_run = 0;
                    }
                }





            }else if(std::strcmp(mode.c_str(), MODE_SHELF2) == 0){

//                compute_leg_pose();

            }





            scan_get_data = false;

        }else{
            suspend.sleep(sleep_time);

        }

    }

    if(enable_control&&(control_finished_cnt>0)){

        const char* final_cmd_vel = R"(
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
)";


        namespace sp = subprocess;
        auto p = sp::Popen(
                {"bash", "-c", final_cmd_vel}, sp::output{sp::PIPE},
                sp::error{sp::PIPE}, sp::defer_spawn{true});
        //   auto p = sp::Popen({"bash", "-c", cmd_string},
        //   sp::shell{true}, sp::defer_spawn{true});

        p.start_process();
    }


    return 0;
}