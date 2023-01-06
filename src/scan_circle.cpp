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

#include <RansacLib/ransac.h>
/*


 */

struct Point{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float r = 0.0;
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

bool circleLeastFit(const std::vector<Point> &points, float &center_x, float &center_y, float &radius)
{
    center_x = 0.0f;
    center_y = 0.0f;
    radius = 0.0f;
    if (points.size() < 3)
    {
        return false;
    }

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = points.size();
    for (int i = 0; i < N; i++)
    {
        double x = points[i].x;
        double y = points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    center_x = a / (-2);
    center_y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;
    return true;
}

//float NormaliseAngle(float angle, float mean){
//    while( std::abs(angle - mean) > M_PI)
//}

// find circle
void FindCircle(std::vector<Point>& points,float radius, float edge_radius, float edge_range_offset, float & center_x, float & center_y, float& error_mean){

    std::sort(points.begin(),points.end(), [](auto& v1, auto& v2){ return v1.r < v2.r; });


    float angle_mean = std::atan2(points[0].y,points[0].x ),angle_sum = 0.0;
    PLOGD << "check angle_mean: " << angle_mean << std::endl;

    int valid_num = 0;
    for(auto&p : points){
        float center_angle = std::atan2(p.y,p.x );
//        std::cout << "angle: " << center_angle <<", ";

        if(p.r < points[0].r + edge_radius){
            angle_sum += std::abs(center_angle - angle_mean) < M_PI ? center_angle : (center_angle + (center_angle - angle_mean) > 0.0 ? -M_PI*2: M_PI*2);
            valid_num++;

        }else{
//            break;

        }
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

    edge_range /= float(egde_num);

    edge_range += radius;


    float cx = ux*(edge_range);
    float cy = uy*(edge_range);

    center_x = ux*(edge_range);
    center_y = uy*(edge_range);

//    cx = points[0].x + ux*(radius);
//    cy = points[0].y + uy*(radius);

    float error_sum = 0.0 ;


    error_sum = 0.0;
    std::cout << "check error:\n";
    for(int i = 0 ; i <valid_num;i++){
        auto& p = points[i];
        float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
        std::cout << d << ", ";
        error_sum += std::abs(d- radius) ;
    }
    error_mean = error_sum/float(valid_num);
    PLOGD << "check cx: " << cx << ", cy: " << cy << std::endl;

    PLOGD << "check error_mean: " << error_mean << std::endl;
    return  ;





    PLOGD << "update cx cy" << std::endl;
    float update_step = 0.001;

    int iteration_step = 50;
    float update_step_decay = update_step/(2*iteration_step);

    float dx = 0.0, dy = 0.0;
    for(int j = 0; j < iteration_step;j++){
        dx = 0.0;
        dy = 0.0;
        for(int i = 0 ; i <valid_num;i++){
            auto& p = points[i];

            float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
            float e = radius -d;
//            PLOGD << "d: " << d << ", e: " << e<< std::endl;

//            e = std::abs(e) > 0.02? 1.0*e: 2.0*e;
//            dx += e*update_step*ux;
//            dy += e*update_step*uy;
//            e = 0.005/e;
            e = std::abs(e) > 0.02? 10.0* e: 15.0*e;
            dx +=  e*update_step*std::abs(p.x - cx);
            dy +=  e*update_step*std::abs(p.y - cy);

//            PLOGD << "dx: " << dx << ", dy: " << dy<< std::endl;

        }
//        PLOGD << "dx: " << dx << ", dy: " << dy<< std::endl;

        cx += dx;
        cy += dy;
        update_step -= update_step_decay;

    }
    error_sum = 0.0;
    for(int i = 0 ; i <valid_num;i++){
        auto& p = points[i];
        float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
        std::cout << d << ", ";
        error_sum += std::abs(d- radius) ;
    }
    PLOGD << "check cx: " << cx << ", cy: " << cy << std::endl;

    PLOGD << "check error: " << error_sum/float(valid_num) << std::endl;

    center_x = cx;
    center_y = cy;
#if 0
    circleLeastFit(points,center_x,cx, cy);
    error_sum = 0.0;
    for(int i = 0 ; i <valid_num;i++){
        auto& p = points[i];
        float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
        std::cout << d << ", ";
        error_sum += std::abs(d- radius) ;
    }
    PLOGD << "check cx: " << cx << ", cy: " << cy << std::endl;

    PLOGD << "check error: " << error_sum/float(valid_num) << std::endl;
#endif
}


struct DetectTarget{
    // x,y,match_error
    float pose_in_laser[2];
    float pose_in_base[2];

    float match_error;
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

    const char* circle_radius_param = "circle_radius";
    const char* circle_edge_radius_param = "circle_edge_radius";
    const char* circle_min_num_in_radius_param = "circle_min_num_in_radius";
    const char* circle_edge_range_offset_param = "circle_edge_range_offset";

    float split_dist_x = 0.08;
    float split_dist_y = 0.08;
    float split_dist_r = 0.05;

    int min_point_num_in_seg = 10;
    const char* split_dist_x_param = "split_dist_x";
    const char* split_dist_y_param = "split_dist_y";
    const char* split_dist_r_param = "split_dist_r";

    const char* min_point_num_in_seg_param = "min_point_num_in_seg";


    float control_target_angle = 0.0;
    float control_target_distance = 0.5;
    const char* control_target_angle_param = "control_target_angle";
    const char* control_target_distance_param = "control_target_distance";




    // control command
    const char* status_param = "status";
    const char* run_param = "run";
    int run_command = 0;
    int run_command_last = 0;
    int start_run = 0;

    //tf
    std::string base_frame = "base_link";
    std::string odom_frame = "odom";
    std::string laser_frame = "base_laser";
    std::string map_frame = "map";
    tf::TransformListener tl_;
    transform::Transform2d tf_base_laser;
    tf::StampedTransform transform;
    bool tf_get_base_laser = false;



    nh_private.getParam(mode_param, mode);

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

    nh_private.getParam(split_dist_x_param, split_dist_x);
    nh_private.getParam(split_dist_y_param, split_dist_y);
    nh_private.getParam(split_dist_r_param, split_dist_r);

    nh_private.getParam(min_point_num_in_seg_param, min_point_num_in_seg);


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

        }

        if(start_run == -1){
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
                FindCircle(filter_points_segments[i], circle_radius,circle_edge_radius,circle_edge_range_offset ,find_result[i].pose_in_laser[0],find_result[i].pose_in_laser[1],find_result[i].match_error);
                tf_base_laser.mul(find_result[i].pose_in_laser, 1, find_result[i].pose_in_base);

            }

            if(find_result.empty()){
                continue;
            }

            std::sort(find_result.begin(),find_result.end(),[](auto& v1,auto& v2){
                return v1.match_error < v2.match_error;
            });
            PLOGD << "control_target_angle: " << control_target_angle << std::endl;
            PLOGD << "control_target_distance: " << control_target_distance << std::endl;
            PLOGD << "target pose in base ,  " << find_result[0].pose_in_base[0] << ", " << find_result[0].pose_in_base[1] << std::endl;


            marker_array_msg.markers[0].header.stamp = scan_time;
            marker_array_msg.markers[0].header.frame_id.assign(laser_frame);
            marker_array_msg.markers[0].pose.position.x =find_result[0].pose_in_laser[0];
            marker_array_msg.markers[0].pose.position.y =find_result[0].pose_in_laser[1];
            marker_array_msg.markers[0].id = 1;

            cloud_target_pub.publish(marker_array_msg);



            float angle_diff = std::atan2(find_result[0].pose_in_base[1],find_result[0].pose_in_base[0] );

            angle_diff -= control_target_angle;

            float distance_diff = std::sqrt(find_result[0].pose_in_base[0]*find_result[0].pose_in_base[0] + find_result[0].pose_in_base[1]*find_result[0].pose_in_base[1] );


            distance_diff -= control_target_distance;





            cmd_vel_msg.angular.z = (std::abs(angle_diff) > 0.01) ? 0.5*   (angle_diff>0.0 ? 0.1:-0.1): 0.0;


            if(std::abs(distance_diff)<0.01 ||   (std::abs(angle_diff) > 0.02) ){

                cmd_vel_msg.linear.x = 0.0;

            }else{
                if(find_result[0].pose_in_base[0] > 0.0 ){

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
            PLOGD << "angle_diff: " << angle_diff << std::endl;
            PLOGD << "distance_diff: " << distance_diff << std::endl;

            PLOGD << "cmd_vel_msg.angular.z : " <<   cmd_vel_msg.angular.z  << std::endl;
            PLOGD << "cmd_vel_msg.linear.x : " <<  cmd_vel_msg.linear.x << std::endl;
            cmd_vel_pub.publish(cmd_vel_msg);

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