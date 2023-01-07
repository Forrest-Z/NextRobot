//
// Created by waxz on 23-1-4.
//

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
#include "sensor/laser_scan.h"
#include "common/clock_time.h"

#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

//#include <autodiff/forward/real.hpp>
//#include <autodiff/forward/real/eigen.hpp>

#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>
/*


 */

#include "pid.h"



struct Point{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float r = 0.0;
};

autodiff::var
circle_opt_fnd(const autodiff::ArrayXvar& x, const std::vector<Point>& points,int point_num, float radius_2)
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

    const std::vector<Point>& points;
    int point_num;
    float radius = 0.1;
    float radius_2 = radius;


    CircleCostFunction(const std::vector<Point>& t_points, int t_point_num, float t_radius):points(t_points),point_num(t_point_num), radius(t_radius),radius_2(radius*radius){
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
                        std::vector<Point>& filter_points, int& filter_point_num,
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
                        std::vector<Point>& filter_points_laser, std::vector<Point>& filter_points_base, int& filter_point_num,
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


/*
 find
 */
void LaserScanSegment( std::vector<Point>& filter_points, std::vector<std::vector<Point>>& segments, int filter_point_num, float split_dist_x,float split_dist_y,float split_dist_r, int min_point_in_seg_num
) {



    // split to segments
    segments.clear();
    std::vector<Point> segment;
    bool process_done = false;
    PLOGD << "filter_point_num " << filter_point_num << std::endl;

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

                    if((j == filter_point_num-1)){
                        segment.emplace_back(filter_points[l]);
                    }
                    if(segment.size() > min_point_in_seg_num){
                        segments.emplace_back(std::move(segment));
                        segment.clear();
                    }
                }

                segment.emplace_back(filter_points[l]);

            }

            process_done = true;

        }
        if(process_done){
            break;
        }
    }
#if 0

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
void FindCircle(std::vector<Point>& points,float radius, float edge_radius, float edge_range_offset, int min_point_num, float & center_x, float & center_y, float& error_mean){

    std::sort(points.begin(),points.end(), [](auto& v1, auto& v2){ return v1.r < v2.r; });


    float angle_mean = std::atan2(points[0].y,points[0].x ),angle_sum = 0.0;
    PLOGD << "check angle_mean: " << angle_mean << std::endl;

    int valid_num = 0;
    float min_r = points[0].r -0.02, max_r = points[0].r + edge_radius;

    for(auto&p : points){
        float center_angle = std::atan2(p.y,p.x );
//        std::cout << "angle: " << center_angle <<", ";

        if( (p.r < max_r )&&(p.r > min_r) ){
            angle_sum += std::abs(center_angle - angle_mean) < M_PI ? center_angle : (center_angle + (center_angle - angle_mean) > 0.0 ? -M_PI*2: M_PI*2);
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
        float center_angle = std::atan2(p.y,p.x );
//        PLOGD<< "angle: " << center_angle <<", x : " << p.x << ", y: " << p.y << "\n"  ;

        center_angle = std::abs(center_angle - angle_mean) < M_PI ? center_angle : (center_angle + (center_angle - angle_mean) > 0.0 ? -M_PI*2: M_PI*2);

        if(std::abs(center_angle - angle_mean)<angle_offset){
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


struct DetectTarget{
    // x,y,match_error
    float pose_in_laser[2];
    float pose_in_base[2];

    float match_error;
};

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
    const char* MODE_SHELF = "shelf";



    float sleep_time = 50.0;
    const char* sleep_time_param = "sleep_time";

    float tf_wait_time = 0.08;
    const char* tf_wait_time_param = "sleep_time";

    float range_max = 30.0;
    const char* range_max_param = "range_max";
    float range_min = 1.0;
    const char* range_min_param = "range_min";

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


    float rotate_pid_control_kp = 0.1;
    float rotate_pid_control_ki = 0.0;
    float rotate_pid_control_kd = 0.0;

    const char* rotate_pid_control_kp_param = "rotate_pid_control_kp";
    const char* rotate_pid_control_ki_param = "rotate_pid_control_ki";
    const char* rotate_pid_control_kd_param = "rotate_pid_control_kd";

    float forward_pid_control_kp = 0.1;
    float forward_pid_control_ki = 0.0;
    float forward_pid_control_kd = 0.0;

    const char* forward_pid_control_kp_param = "forward_pid_control_kp";
    const char* forward_pid_control_ki_param = "forward_pid_control_ki";
    const char* forward_pid_control_kd_param = "forward_pid_control_kd";

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
    tf::StampedTransform transform;
    bool tf_get_base_laser = false;


    auto load_params = [&]{
        nh_private.getParam(mode_param, mode);
        nh_private.getParam(max_fit_error_param, max_fit_error);
        nh_private.getParam(sleep_time_param, sleep_time);
        nh_private.getParam(tf_wait_time_param, tf_wait_time);
        nh_private.getParam(range_max_param, range_max);
        nh_private.getParam(range_min_param, range_min);
        nh_private.getParam(scan_point_jump_param, scan_point_jump);
        nh_private.getParam(scan_noise_angle_param, scan_noise_angle);

        nh_private.getParam(box_filter_min_x_param, box_filter_min_x);
        nh_private.getParam(box_filter_max_x_param, box_filter_max_x);
        nh_private.getParam(box_filter_min_y_param, box_filter_min_y);
        nh_private.getParam(box_filter_max_y_param, box_filter_max_y);

        nh_private.getParam(circle_radius_param, circle_radius);
        nh_private.getParam(circle_edge_radius_param, circle_edge_radius);
        nh_private.getParam(circle_min_num_in_radius_param, circle_min_num_in_radius);

        nh_private.getParam(circle_min_point_num_param, circle_min_point_num);


        nh_private.getParam(split_dist_x_param, split_dist_x);
        nh_private.getParam(split_dist_y_param, split_dist_y);
        nh_private.getParam(split_dist_r_param, split_dist_r);

        nh_private.getParam(min_point_num_in_seg_param, min_point_num_in_seg);

        nh_private.getParam(control_target_angle_param, control_target_angle);
        nh_private.getParam(control_target_distance_param, control_target_distance);
        nh_private.getParam(enable_control_param, enable_control);

        nh_private.getParam(rotate_pid_control_kp_param, rotate_pid_control_kp);
        nh_private.getParam(rotate_pid_control_ki_param, rotate_pid_control_ki);
        nh_private.getParam(rotate_pid_control_kd_param, rotate_pid_control_kd);
        nh_private.getParam(forward_pid_control_kp_param, forward_pid_control_kp);
        nh_private.getParam(forward_pid_control_ki_param, forward_pid_control_ki);
        nh_private.getParam(forward_pid_control_kd_param, forward_pid_control_kd);
    };

    load_params();



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
        scan_handler.getLocalPoints(msg->ranges, msg->angle_min, msg->angle_increment, range_min, range_max);
        scan_get_data = scan_handler.range_valid_num > 10;
    };
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laser_cb);


    auto odom_cb = [&](const  nav_msgs::OdometryConstPtr &msg){

    };
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odom_cb);



    const char* cmd_vel_topic = "/cmd_vel";
    const char* cloud_target_topic = "detect_target";

    ros::Publisher cmd_vel_pub =  nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

    geometry_msgs::Twist cmd_vel_msg;

    ros::Publisher cloud_target_pub = nh_private.advertise<visualization_msgs::MarkerArray>(cloud_target_topic,1);


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


    std::vector<Point> filter_points_laser(100);
    std::vector<Point> filter_points_base(100);
    std::vector<std::vector<Point>> filter_points_segments;


    std::vector<float> points_in_base(500);

    int filter_point_num = 0;


    PID rotate_pid(0.05, 0.1, -0.1, rotate_pid_control_kp, rotate_pid_control_kd, rotate_pid_control_ki);
    common::Time rotate_pid_t1 = common::FromUnixNow();

    PID forward_pid(0.05, 0.1, -0.1, forward_pid_control_kp, forward_pid_control_kd, forward_pid_control_ki);

    float rotate_dt = 50.0;



    while (ros::ok()){
        nh_private.getParam(run_param,run_command);

#if 1
        if(run_command == 0){
            run_command_last = run_command;
            suspend.sleep(1000.0);
            continue;
        }
        if(run_command && !run_command_last){
            run_command_last = run_command;

            start_run = 1;
            control_finished_cnt = 0;

        }

        if(start_run == -1 || start_run == 0){
            suspend.sleep(1000.0);
            continue;
        }

#endif

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

                } catch (tf::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                    continue;
                }
            }

            PLOGD << "tf_base_laser: " <<  tf_base_laser << std::endl;

            tf_base_laser.mul(scan_handler.local_xy_points, scan_handler.range_valid_num, points_in_base);

            PLOGD << "range_valid_num: " <<  scan_handler.range_valid_num << std::endl;
#if 0
            for(int i = 0; i < scan_handler.range_valid_num;i++){
                std::cout << "== "<< i << ", " << scan_handler.local_xy_points[i+i] << ", " << scan_handler.local_xy_points[i+i+1]  <<"\n";

            }
#endif
            LaserScanBoxFilter(scan_handler.local_xy_points, points_in_base, scan_handler.range_valid_num,

                               filter_points_laser,filter_points_base, filter_point_num,box_filter_min_x,box_filter_max_x,box_filter_min_y,box_filter_max_y );



            PLOGD << "filter_point_num: " << filter_point_num << std::endl;
#if 0
            for(int i = 0; i < filter_point_num;i++){
                std::cout << "** "<< i << ", " << filter_points_laser[i].x << ", " << filter_points_laser[i].y  <<"\n";

            }
#endif

            LaserScanSegment(filter_points_laser,filter_points_segments, filter_point_num,split_dist_x, split_dist_y, split_dist_r, min_point_num_in_seg);


            std::vector<DetectTarget> find_result(filter_points_segments.size());
            for(int i = 0 ; i <filter_points_segments.size(); i++ ){
                FindCircle(filter_points_segments[i], circle_radius,circle_edge_radius,circle_edge_range_offset, circle_min_point_num ,find_result[i].pose_in_laser[0],find_result[i].pose_in_laser[1],find_result[i].match_error);
                tf_base_laser.mul(find_result[i].pose_in_laser, 1, find_result[i].pose_in_base);

            }

            if(find_result.empty()){
                continue;
            }

            std::sort(find_result.begin(),find_result.end(),[](auto& v1,auto& v2){
                return (std::abs(v1.pose_in_base[0])+std::abs(v1.pose_in_base[1])) < ( std::abs(v2.pose_in_base[0]) + std::abs(v2.pose_in_base[1]));
            });

            int best_result_id = 0;
            while(find_result[best_result_id].match_error > max_fit_error){

                best_result_id++;
                if(best_result_id == find_result.size()){
                    best_result_id = -1;
                    break;
                }
            }

            if(best_result_id == -1){
                PLOGD << "find circle fail " << std::endl;

                continue;
            }else{

            }
            auto best_result = find_result[best_result_id];

            PLOGD << "control_target_angle: " << control_target_angle << std::endl;
            PLOGD << "control_target_distance: " << control_target_distance << std::endl;
            PLOGD << "target pose in base ,  " << best_result.pose_in_base[0] << ", " << best_result.pose_in_base[1] << std::endl;


            marker_array_msg.markers[0].header.stamp = scan_time;
            marker_array_msg.markers[0].header.frame_id.assign(laser_frame);
            marker_array_msg.markers[0].pose.position.x =best_result.pose_in_laser[0];
            marker_array_msg.markers[0].pose.position.y =best_result.pose_in_laser[1];
            marker_array_msg.markers[0].id = 1;

            cloud_target_pub.publish(marker_array_msg);



            float marker_angle = std::atan2(best_result.pose_in_base[1],best_result.pose_in_base[0] );

            float angle_diff = marker_angle - control_target_angle;

            float marker_distance = std::sqrt(best_result.pose_in_base[0]*best_result.pose_in_base[0] + best_result.pose_in_base[1]*best_result.pose_in_base[1] );
            float distance_diff = marker_distance - control_target_distance;

            PLOGD << "angle_diff: " << angle_diff << std::endl;
            PLOGD << "distance_diff: " << distance_diff << std::endl;

            if(!enable_control){
                continue;
            }

            auto now = common::FromUnixNow();
            rotate_dt = common::ToMillSeconds(now - rotate_pid_t1) * 0.001;
            rotate_pid_t1 = now;
            rotate_dt = (rotate_dt > 0.1) ? 0.1: rotate_dt;


            double rotate_pid_inc = rotate_pid.calculate(angle_diff, 0.0);
            PLOGD << "rotate_pid_inc: " << rotate_pid_inc << std::endl;

            if(std::abs(rotate_pid_inc) < 0.01){
                rotate_pid_inc = 0.0;
            }
            cmd_vel_msg.angular.z = rotate_pid_inc;


//            if(std::abs(angle_diff) < 0.02)
            {
                float target_pose_x = distance_diff * std::cos(marker_angle);

                target_pose_x = std::abs(target_pose_x) > 0.03 ? 0.03 * (target_pose_x>0.0 ? 1.0:-1.0 ):target_pose_x;

                double forward_pid_inc = forward_pid.calculate(target_pose_x, 0.0);

                PLOGD << "forward_pid_inc: " << forward_pid_inc << std::endl;
                if(std::abs(forward_pid_inc) < 0.01){
                    forward_pid_inc = 0.0;
                }
                cmd_vel_msg.linear.x = forward_pid_inc;

            }


#if 0
            cmd_vel_msg.angular.z = (std::abs(angle_diff) > 0.01) ? 0.5* (angle_diff>0.0 ? 0.1:-0.1): 0.0;
            if(std::abs(distance_diff)<0.01 ||   (std::abs(angle_diff) > 0.02) ){

                cmd_vel_msg.linear.x = 0.0;

            }else{
                if(best_result.pose_in_base[0] > 0.0 ){

                    if(distance_diff>0.0){
                        cmd_vel_msg.linear.x = 0.1;

                    }else{
                        cmd_vel_msg.linear.x = -0.1;

                    }
                }else{
                    if(distance_diff>0.0){
                        cmd_vel_msg.linear.x = -0.1;

                    }else{
                        cmd_vel_msg.linear.x = 0.1;

                    }
                }
            }


#endif


            PLOGD << "cmd_vel_msg.angular.z : " <<   cmd_vel_msg.angular.z  << std::endl;
            PLOGD << "cmd_vel_msg.linear.x : " <<  cmd_vel_msg.linear.x << std::endl;
            cmd_vel_pub.publish(cmd_vel_msg);

            if(std::abs(distance_diff)<0.01 && std::abs(angle_diff) < 0.01){
                PLOGD << "control finished test : " <<  control_finished_cnt << std::endl;

                control_finished_cnt++;
                if(control_finished_cnt > 10){
                    start_run = 0;
                }
            }

#if 0
            float tx = filter_points_laser[0].x + circle_radius;
            float ty = filter_points_laser[0].y;
            for(int i = 0; i < filter_point_num;i++){
                float r = std::sqrt((filter_points_laser[i].x - tx)*(filter_points_laser[i].x - tx) + (ty  - filter_points_laser[i].y) *(ty - filter_points_laser[i].y) );
                std::cout << filter_points_laser[i].x << ", " << filter_points_laser[i].y <<  ", " << filter_points_laser[i].r << ", "<< r << "\n";

            }
#endif
            // find edge point
            // radius filter
            // compute target center

            scan_get_data = false;

        }else{
            suspend.sleep(sleep_time);

        }

    }


    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_msg.linear.x = 0.0;
    for(int i = 0; i < 5;i++){
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();
        suspend.sleep(500.0);
    }


    return 0;
}