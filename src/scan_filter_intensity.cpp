//
// Created by waxz on 22-10-11.
//

#include <iostream>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/LaserScan.h"
#include "xmlrpcpp/XmlRpc.h"

#include <thread>


#include "transform/transform.h"
#include "sensor/laser_scan.h"

#include "pose_solver/ceres_scan_to_marker.h"

#include "nlohmann/json.hpp"


#include "common/clock_time.h"
#include <fstream>
/*

 use LaserScan data to locate robot with predefined marker_position

 there are two problems to solve

 the fist one is to determine marker_position given LaserScan and base_link movement

 the second is to compute base_link position

 this is kind of like slam problem, marker_position and base_link_position should be solved at the same time


 */


int main(int argc, char** argv){

    //==== ros
    ros::init(argc, argv, "scan_filter_intensity");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    XmlRpc::XmlRpcValue marker_points_in_map;
    std::string marker_points_in_map_xml_str;
    std::string marker_points_in_map_json_str;



    std::string marker_param = "marker_points_in_map";
    float scan_filter_radius = 0.3;
    std::string scan_filter_radius_param = "scan_filter_radius";

    float min_intensity = 600.0;
    std::string min_intensity_param = "min_intensity";

    int min_light_num = 3;
    std::string min_light_num_param = "min_light_num";

    int max_light_num = 3;
    std::string max_light_num_param = "max_light_num";

    bool calib_or_loc = true;
    bool calib_ok = false;

    std::string calib_or_loc_param = "calib_or_loc";

    std::string calib_file = "calib_file.yaml";

    std::string calib_file_param = "calib_file";

    bool tf_broadcast= false;

    std::string tf_broadcast_param = "tf_broadcast";

    double solve_interval = 50.0;
    std::string solve_interval_param = "solve_interval";

    double sequence_clear_interval = 5e3;
    std::string sequence_clear_interval_param = "sequence_clear_interval";


    std::vector<std::array<float,3>> marker_points;
    std::vector<float> marker_points_array_2d;
    std::vector<float> marker_points_offset_array_2d;

    std::vector<float> marker_points_array_2d_in_laser;
    std::vector<float> marker_points_offset_array_2d_in_laser;


    std::vector<double> marker_points_flat_parameter;
    std::vector<std::array<double,3>> marker_points_parameter;


    std::vector<std::array<float,3>> marker_points_in_laser;

    int CERES_SEQUENCE_NUM = 10;
    std::string CERES_SEQUENCE_NUM_param = "CERES_SEQUENCE_NUM";



    const int CERES_Match_arkerToScan = 0;
    const int CERES_Match_ScanToMarker = 1;

    int ceres_mode = CERES_Match_ScanToMarker;
    // odom_base_tf, point_cluster,laser_pose_inv_tf, laser_pose_inv_vector,

    std::deque<std::tuple<transform::Transform2d, std::vector<std::vector<std::array<float,3>>>  , transform::Transform2d, std::array<double,3> ,common::Time,

    std::vector<std::vector<float>>
    >  > ceres_sequence;


    float marker_radius = 0.02;



    bool control_command_on = false;
    bool control_command_on_last = false;

    std::string control_command_on_param = "run";

    int control_status = 0;
    std::string control_status_param = "status";

    std::string amcl_tf_broadcast = "/amcl/tf_broadcast";

    std::string amcl_tf_broadcast_param = "amcl_tf_broadcast";

    double ceres_constrains_weight_x = 1.0;
    double ceres_constrains_weight_y = 1.0;
    double ceres_constrains_weight_yaw = 1.0;
    double ceres_fit_weight = 1.0;


    std::string ceres_constrains_weight_x_param = "ceres_constrains_weight_x";
    std::string ceres_constrains_weight_y_param = "ceres_constrains_weight_y";
    std::string ceres_constrains_weight_yaw_param = "ceres_constrains_weight_yaw";
    std::string ceres_fit_weight_param = "ceres_fit_weight";


    double solve_fail_time = 1000.0;
    std::string solve_fail_time_param = "solve_fail_time";


    XmlRpc::XmlRpcValue marker_points_cluster_xml;

    std::vector<std::vector<std::array<float,3>>> marker_points_clusters;
    std::vector<std::array<float,3>> marker_points_cluster;
    std::string marker_points_cluster_param = "marker_points_in_map_cluster";


    // detect_free: publish map odom tf
    // detect_free_init : publish initialpose
    std::string detect_mode = "detect_free";
    const std::string DETECT_MODE_FREE = "detect_free";
    const std::string DETECT_MODE_FREE_INIT = "detect_free_init";

    std::string detect_mode_param = "detect_mode";

    std::array<float,3> pints;

    if (nh_private.hasParam(marker_param)){
        nh_private.getParam(marker_param,marker_points_in_map);
        nh_private.getParam(scan_filter_radius_param,scan_filter_radius);
        nh_private.getParam(min_intensity_param,min_intensity);
        nh_private.getParam(min_light_num_param,min_light_num);
        nh_private.getParam(max_light_num_param,max_light_num);
        nh_private.getParam(calib_or_loc_param,calib_or_loc);
        nh_private.getParam(calib_file_param,calib_file);
        nh_private.getParam(CERES_SEQUENCE_NUM_param,CERES_SEQUENCE_NUM);
        nh_private.getParam(tf_broadcast_param,tf_broadcast);

        nh_private.getParam(control_command_on_param,control_command_on);

        nh_private.getParam(amcl_tf_broadcast_param,amcl_tf_broadcast);
        nh_private.getParam(solve_fail_time_param,solve_fail_time);

        nh_private.getParam(ceres_constrains_weight_x_param,ceres_constrains_weight_x);
        nh_private.getParam(ceres_constrains_weight_y_param,ceres_constrains_weight_y);
        nh_private.getParam(ceres_constrains_weight_yaw_param,ceres_constrains_weight_yaw);
        nh_private.getParam(ceres_fit_weight_param,ceres_fit_weight);

        nh_private.getParam(solve_interval_param,solve_interval);

        nh_private.getParam(detect_mode_param,detect_mode);


        nh_private.getParam(marker_points_cluster_param,marker_points_cluster_xml);
        {

            std::cout << "get " << marker_points_cluster_param << "\n " << marker_points_cluster_xml.toXml() << std::endl;

            auto t = marker_points_cluster_xml.getType();
            if(t!= XmlRpc::XmlRpcValue::TypeArray){
                std::cerr << "get " << marker_points_cluster_param << " fail: " <<  "wrong type: XmlRpc::XmlRpcValue::TypeArray != " << t <<marker_points_cluster_xml.toXml() << std::endl;
                return 0;

            }
            marker_points_clusters.clear();
            for(int i = 0 ; i <marker_points_cluster_xml.size();i++ ){
                marker_points_cluster.clear();

                for(int j = 0 ; j < marker_points_cluster_xml[i].size(); j++){
                    std::cout <<"check 1 :\n" <<marker_points_cluster_xml[i][j].toXml() << std::endl;


                    int L = marker_points_cluster_xml[i][j].size()>3 ? 3:marker_points_cluster_xml[i][j].size();
                    std::cout << "L = " << L << std::endl;

    #if 1
                    for(int l = 0 ; l < L;l++){
                        pints[l] = static_cast<double >(marker_points_cluster_xml[i][j][l]) ;

                    }
#endif
                    marker_points_cluster.push_back(pints);


                }




                marker_points_clusters.push_back(marker_points_cluster);
            }
        }


        marker_points_in_map_xml_str = marker_points_in_map.toXml();
        auto t = marker_points_in_map.getType();
         if(t!= XmlRpc::XmlRpcValue::TypeArray){
             std::cerr << "get " << marker_param << " fail: " <<  "wrong type: XmlRpc::XmlRpcValue::TypeArray != " << t <<marker_points_in_map_xml_str << std::endl;
             return 0;

         }

         int N= marker_points_in_map.size();
         for(int i = 0; i < N;i++){
             auto &e = marker_points_in_map[i];
             if( e.getType()!= XmlRpc::XmlRpcValue::TypeArray){
                 std::cerr << "get " << marker_param << " fail: " <<  "wrong type: XmlRpc::XmlRpcValue::TypeArray != " << e.getType() <<e.toXml() << std::endl;
                 return 0;

             }

             int M = e.size()>3 ? 3:e.size();
             for(int j = 0 ; j < M;j++){
                 pints[j] = static_cast<double >(e[j]) ;
             }
             marker_points.emplace_back(pints);
         }



    }else{
        std::cerr << "get " << marker_param << " fail" << std::endl;

        return 0;

    }
    std::cerr << "get  marker_points ok" << std::endl;

    marker_points_array_2d.resize(marker_points.size() + marker_points.size());
    marker_points_array_2d_in_laser.resize(marker_points.size() + marker_points.size());
    marker_points_offset_array_2d_in_laser.resize(marker_points.size() + marker_points.size());

    marker_points_in_laser = marker_points;
    for(int i = 0; i < marker_points.size();i++){
        std::cout << marker_points[i][0] << ", " << marker_points[i][1] << ", " << marker_points[i][2] << std::endl;
        marker_points_array_2d[i+i] =  marker_points[i][0];
        marker_points_array_2d[i+i +1] =  marker_points[i][1];

    }


    common::simple_cast(marker_points_flat_parameter, marker_points_array_2d);
    common::simple_cast(marker_points_parameter, marker_points);

    std::vector<pose_solver::PointsToGrid> ceres_grid_vec;
    std::vector< ceres::BiCubicInterpolator<pose_solver::PointsToGrid>> ceres_interpolator_vec;


    std::cerr << " marker_points_clusters.size = " << marker_points_clusters.size() << std::endl;

    {

        for(int i = 0; i < marker_points_clusters.size();i++){
            std::vector<float> points_for_grid;

            std::cerr << " marker_points_clusters[i].size() = " << marker_points_clusters[i].size()<< std::endl;

            points_for_grid.resize(marker_points_clusters[i].size() + marker_points_clusters[i].size());
            for(int j = 0; j < marker_points_clusters[i].size(); j++){
                points_for_grid[j+j] = marker_points_clusters[i][j][0];
                points_for_grid[j+j+1] = marker_points_clusters[i][j][1];

            }
            std::cerr << " points_for_grid.size() = " << points_for_grid.size()<< std::endl;

            ceres_grid_vec.emplace_back(points_for_grid);
            ceres_interpolator_vec.emplace_back(ceres_grid_vec.back());

        }

    }



    geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
    initialpose_msg.header.frame_id = "map";
    initialpose_msg.header.stamp = ros::Time::now();
//    initialpose_msg.pose.pose.position.x = json_process_data["instructionData"]["positionX"];
//    initialpose_msg.pose.pose.position.y = json_process_data["instructionData"]["positionY"];
//    initialpose_msg.pose.pose.position.z = 0.0; // json_process_data["instructionData"]["positionX"];
//    initialpose_msg.pose.pose.orientation.x = json_process_data["instructionData"]["orientationX"];
//    initialpose_msg.pose.pose.orientation.y =  json_process_data["instructionData"]["orientationY"];
//    initialpose_msg.pose.pose.orientation.z = json_process_data["instructionData"]["orientationZ"];
//    initialpose_msg.pose.pose.orientation.w = json_process_data["instructionData"]["orientationW"];

    initialpose_msg.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
    initialpose_msg.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
    initialpose_msg.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;


    //===
    std::string fixed_frame = "/map";
    std::string odom_frame = "/odom";
    std::string base_frame = "/base_link";

    tf::TransformListener tl_;
    tf::TransformBroadcaster tf_br;
    tf::StampedTransform pub_tf_stamped;
    bool map_odom_tf_computed = false;
    bool map_odom_tf_computed_last = false;


    ros::Duration transform_tolerance_;
    transform_tolerance_.fromSec(1.0);


    //tf2
#if 0
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
#endif

    tf::Transform transform_;
    tf::Transform identity_transform_;

    bool tf_transform_received = false;
    bool tf_static_transform_received = false;

    bool scan_received = false;

    tf::StampedTransform transform;



    sensor::ScanToPoints scan_handler;

    scan_handler.filterOption.radius = scan_filter_radius;
    scan_handler.filterOption.min_intensity = min_intensity;
    scan_handler.filterOption.min_light_num = min_light_num;
    scan_handler.filterOption.max_light_num = max_light_num;

    transform::Transform2d transform2D;
    transform::Transform2d transform2D_inv;

    transform::Transform2d base_to_laser_tf;
    transform::Transform2d odom_to_base_tf;
    transform::Transform2d map_to_base_tf;



    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_filtered_intensity",10);
    sensor_msgs::LaserScan scan_msg;

    ros::Publisher initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
    ros::Publisher solved_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/solved_pose",10);

    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker  marker_msg;
    marker_msg.header.frame_id = "map";//fixed_frame;
    marker_msg.type = visualization_msgs::Marker::CYLINDER;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.05;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;
    marker_msg.header.stamp = ros::Time();
    marker_msg.color.a = 0.5; // Don't forget to set the alpha!
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;


    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker",10);


    int marker_num = marker_points.size();
    marker_array_msg.markers.resize(marker_num + marker_num,marker_msg);
    for(int i = 0; i < marker_points.size();i++){
        marker_array_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
        marker_array_msg.markers[i].pose.position.x = marker_points[i][0];
        marker_array_msg.markers[i].pose.position.y = marker_points[i][1];
        marker_array_msg.markers[i].id = i;


        marker_array_msg.markers[i+marker_num].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_array_msg.markers[i+marker_num].pose.position.x = marker_points[i][0];
        marker_array_msg.markers[i+marker_num].pose.position.y = marker_points[i][1];
        marker_array_msg.markers[i+marker_num].id = i+marker_num;

        char s[100];
        sprintf(s, R"(%d : [%.3f,%.3f])",i, marker_points[i][0],marker_points[i][1]);
        marker_array_msg.markers[i+marker_num].text = s;
    }



    common::Time last_scan_time = common::FromUnixNow();
    common::Time last_solve_time = common::FromUnixNow();

    int valid_marker_num = 0;


    bool update_valid_laser_mark = false;




    auto cb = [&]( const sensor_msgs::LaserScanConstPtr &msg) {

        last_scan_time = common::FromUnixNow();

        if(!control_command_on){

            return ;
        }

        scan_handler.getLocalPoints(msg->ranges,msg-> intensities, msg->angle_min,msg->angle_increment,msg->range_max   );

        if(!tf_static_transform_received){
            ROS_INFO("Waiting for static transform. %s to %s ",base_frame.c_str(), base_frame.c_str() );

            try{
//                tl_.waitForTransform(base_frame, msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
                tl_.lookupTransform(base_frame, msg->header.frame_id, ros::Time(0), transform);

#if 0
                // use tf2
                transformStamped  = tfBuffer.lookupTransform(base_frame, msg->header.frame_id, ros::Time(0) );

#endif


                base_to_laser_tf.set(transform.getOrigin().x(),transform.getOrigin().y(),  tf::getYaw(transform.getRotation()));

                tf_static_transform_received = true;
            }catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
//            ros::Duration(1.0).sleep();
                tf_static_transform_received = false;
            }


        }
        // get time
        // lookup tf
        if ((fixed_frame != msg->header.frame_id)and (tf_static_transform_received) ){
            try {
                ROS_INFO("Waiting for dynamic transform. %s to %s ",odom_frame.c_str(), base_frame.c_str() );

#if 0

                //                tl_.waitForTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(10.0));
                tl_.lookupTransform(odom_frame, base_frame, msg->header.stamp, transform);
                odom_to_base_tf.set(transform.getOrigin().x(),transform.getOrigin().y(),  tf::getYaw(transform.getRotation()));

                ROS_INFO("Get dynamic transform. %s to %s ",odom_frame.c_str(), base_frame.c_str() );

#endif



                ROS_INFO("Waiting for dynamic transform. %s to %s ",fixed_frame.c_str(), base_frame.c_str() );


//                tl_.waitForTransform(fixed_frame, base_frame, ros::Time(0), ros::Duration(10.0));
                tl_.lookupTransform(odom_frame, base_frame, msg->header.stamp, transform);
                odom_to_base_tf.set(transform.getOrigin().x(),transform.getOrigin().y(),  tf::getYaw(transform.getRotation()));

                ROS_INFO("Get dynamic transform. %s to %s ",fixed_frame.c_str(), base_frame.c_str() );


                std::cout << "base_to_laser_tf" << base_to_laser_tf << std::endl;
                std::cout << "odom_to_base_tf" << odom_to_base_tf << std::endl;
                ROS_INFO("Waiting for dynamic transform. %s to %s ",fixed_frame.c_str(), msg->header.frame_id.c_str() );


//                tl_.waitForTransform(fixed_frame, msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
                tl_.lookupTransform(fixed_frame, msg->header.frame_id, msg->header.stamp, transform);
                transform_.setOrigin(transform.getOrigin());
                transform_.setRotation(transform.getRotation());

                ROS_INFO("Get dynamic transform. %s to %s ",fixed_frame.c_str(), msg->header.frame_id.c_str() );


                ROS_INFO("Received tf transform.");
                tf_transform_received = true;

                scan_handler.getGlobalPoints();


                transform2D.set(transform_.getOrigin().x(),transform_.getOrigin().y(),  tf::getYaw(transform_.getRotation()));
                std::cerr << "transform2D:\n" << transform2D << std::endl;
                transform2D_inv = transform2D.inverse();
                std::cerr << "transform2D.inv:\n" << transform2D_inv<< std::endl;
                transform2D_inv.mul(marker_points_array_2d,marker_points_array_2d_in_laser);

                std::cerr << "\n==== marker_points_array_2d_in_laser:\n";
                for(int i = 0; i < marker_points.size();i++){
                    std::cout  << "[ " << marker_points_array_2d_in_laser[i+i] << ", " <<  marker_points_array_2d_in_laser[i+i +1]  << "]," ;

                    marker_points_in_laser[i][0] = marker_points_array_2d_in_laser[i+i];
                    marker_points_in_laser[i][1] = marker_points_array_2d_in_laser[i+i+1];

                }

                scan_handler.filterMarker(marker_points_in_laser);
                std::cerr << "====\n";
                scan_msg.header = msg->header;
                scan_msg.intensities = msg->intensities;
                scan_msg.angle_increment = msg->angle_increment;
                scan_msg.range_max = msg->range_max;
                scan_msg.range_min = msg->range_min;
                scan_msg.time_increment = msg->time_increment;
                scan_msg.angle_min = msg->angle_min;
                scan_msg.angle_max = msg->angle_max;

                scan_msg.ranges = scan_handler.ranges_filtered;
                scan_pub.publish(scan_msg);

                valid_marker_num = 0;
                for(int i = 0; i < marker_points.size();i++){
                    char s[100];
                    float mean_x = 0.0, mean_y = 0.0;
                    for(auto&e : scan_handler.filter_cluster_points[i]){
                        mean_x += e[0];
                        mean_y += e[1];
                    }
                    if (scan_handler.filter_cluster_points[i].size() >0){
                        mean_x /= scan_handler.filter_cluster_points[i].size();
                        mean_y /= scan_handler.filter_cluster_points[i].size();
                        valid_marker_num ++;

                    }

                    sprintf(s, R"(%d : [%.3f,%.3f], %zu , [%.3f,%.3f], [%.3f,%.3f])",i, marker_points[i][0],marker_points[i][1],scan_handler.filter_cluster_points[i].size()
                    ,marker_points_array_2d_in_laser[i+i],marker_points_array_2d_in_laser[i+i+1],mean_x, mean_y);
                    marker_array_msg.markers[i+marker_num].text = s;
                }


                marker_pub.publish(marker_array_msg);

                std::cerr << "valid_marker_num " << valid_marker_num << std::endl;


                if(valid_marker_num >=2){
                    std::cerr << "add one node\n";


                    if(!ceres_sequence.empty()){
                        double dur_ms = common::ToMillSeconds(common::FromUnixNow() - std::get<4>( ceres_sequence.back()));
                        if (dur_ms > sequence_clear_interval){
                            std::cerr << "clear ceres_sequence" << std::endl;
                            ceres_sequence.clear();
                        }
                    }
                    if(ceres_mode == CERES_Match_ScanToMarker){

                        std::vector<std::vector<float>> scan_points(scan_handler.filter_cluster_points.size());
                        for(int i = 0 ; i < scan_handler.filter_cluster_points.size();i++){
                            scan_points[i].resize(scan_handler.filter_cluster_points[i].size()+scan_handler.filter_cluster_points[i].size());

                            for(int j = 0 ; j  <scan_handler.filter_cluster_points[i].size();j++){
                                scan_points[i][j+j] = scan_handler.filter_cluster_points[i][j][0];
                                scan_points[i][j+j+1] = scan_handler.filter_cluster_points[i][j][1];

                            }
                        }



                        ceres_sequence.emplace_back( std::make_tuple(odom_to_base_tf, scan_handler.filter_cluster_points, transform2D, std::array<double,3>{transform2D.x(),transform2D.y(),transform2D.yaw()}  ,

                                                                     common::FromUnixNow(),scan_points
                        )     );
                    }else{
                        std::vector<std::vector<float>> scan_points;

                        ceres_sequence.emplace_back( std::make_tuple(odom_to_base_tf, scan_handler.filter_cluster_points, transform2D_inv, std::array<double,3>{transform2D_inv.x(),transform2D_inv.y(),transform2D_inv.yaw()}  ,

                                                                     common::FromUnixNow(),scan_points
                        )     );
                    }


                    update_valid_laser_mark = true;

                }




            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
//            ros::Duration(1.0).sleep();
                tf_transform_received = false;
                update_valid_laser_mark = false;
            }


        }else{
            tf_transform_received = true;
//            scan_handler.getGlobalPoints();

        }


        // transform points

        // get json

        if (tf_transform_received and update_valid_laser_mark) {

            double dur_ms = common::ToMillSeconds(common::FromUnixNow() - last_solve_time);



            if (ceres_sequence.size() > CERES_SEQUENCE_NUM){
                ceres_sequence.pop_front();

            }


            bool to_solve = dur_ms > solve_interval;

//            to_solve = true;

            if (to_solve and (ceres_sequence.size() == CERES_SEQUENCE_NUM) ){
                last_solve_time = common::FromUnixNow();

                if(ceres_mode == CERES_Match_ScanToMarker){


                    // for each marker , create cost function
                    // create flat pointcloud array
                    ceres::Problem problem;

                    auto& init_x = std::get<3>(ceres_sequence.back());
                    auto& pointcloud = std::get<5>(ceres_sequence.back());

                    for(int i = 0 ; i < ceres_grid_vec.size();i++){

                        if (pointcloud[i].empty()) continue;


                        ceres::CostFunction* cost_function =   pose_solver::SimpleAutoDiffBiCubicCost_V2::Create(&(ceres_grid_vec[i]), 1.0, std::get<5>(ceres_sequence.back())[i]);
                        problem.AddResidualBlock(cost_function, nullptr, &(init_x[0]));

                    }
                    ceres::Solver::Options ceres_solver_options;
//
                    ceres_solver_options.minimizer_progress_to_stdout = true;
                    ceres_solver_options.num_threads = 1;
                    ceres_solver_options.function_tolerance = 1e-3;  // Enough for denoising.
                    ceres_solver_options.max_num_iterations = 100;
                    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                    ceres_solver_options.parameter_tolerance = 1e-12;

                    //===
                    ceres_solver_options.minimizer_progress_to_stdout = false;
                    ceres_solver_options.num_threads = 1;
                    ceres_solver_options.function_tolerance = 1e-4;  // Enough for denoising.
                    ceres_solver_options.max_num_iterations = 200;
                    ceres_solver_options.gradient_tolerance = 1e-4;
                    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                    ceres_solver_options.parameter_tolerance = 1e-4;

//        ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // crash
//        ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // crash
                    ceres_solver_options.linear_solver_type = ceres::DENSE_QR;

                    ceres_solver_options.linear_solver_type = ceres::DENSE_SCHUR;


                    ceres::Solver::Summary summary;
                    ceres::Solve(ceres_solver_options, &problem, &summary);
                    std::cout << summary.BriefReport() << '\n';
                    std::cout << "init_x : " << init_x[0] << ", " << init_x[1] << ", " << init_x[2] << std::endl;


                }else{

                    update_valid_laser_mark = false;
                    ceres::Problem problem;

                    ceres::Solver::Options ceres_solver_options;
//
                    ceres_solver_options.minimizer_progress_to_stdout = true;
                    ceres_solver_options.num_threads = 1;
                    ceres_solver_options.function_tolerance = 1e-3;  // Enough for denoising.
                    ceres_solver_options.max_num_iterations = 100;
                    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                    ceres_solver_options.parameter_tolerance = 1e-12;

                    //===
                    ceres_solver_options.minimizer_progress_to_stdout = false;
                    ceres_solver_options.num_threads = 1;
                    ceres_solver_options.function_tolerance = 1e-4;  // Enough for denoising.
                    ceres_solver_options.max_num_iterations = 200;
                    ceres_solver_options.gradient_tolerance = 1e-4;
                    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                    ceres_solver_options.parameter_tolerance = 1e-4;

//        ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // crash
//        ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // crash
                    ceres_solver_options.linear_solver_type = ceres::DENSE_QR;

                    ceres_solver_options.linear_solver_type = ceres::DENSE_SCHUR;


                    for(int i = 1 ;i < CERES_SEQUENCE_NUM ;i++){

                        auto& cluster = std::get<1>(ceres_sequence[i]);
                        auto& init_x = std::get<3>(ceres_sequence[i]);
                        auto& marker_ = marker_points_parameter[i];
                        std::cout << "==== " << i << "\n";
                        std::cout << "init init_x: [ " << init_x[0] << ", " << init_x[1] << ", " << init_x[2] << " ]\n";

                    }
                    for(int j = 0;j < marker_points_parameter.size();j++){
                        auto& marker_ = marker_points_parameter[j];
                        std::cout << "init marker_: [ " << marker_[0] << ", " << marker_[1] << ", " << marker_[2] << " ]\n";
                    }

                    if (calib_or_loc){

                        std::cout << "run calibration function" << std::endl;

                        for(int i = 1 ;i < CERES_SEQUENCE_NUM ;i++){

                            auto& cluster = std::get<1>(ceres_sequence[i]);
                            auto& init_x = std::get<3>(ceres_sequence[i]);
                            auto& init_x_last = std::get<3>(ceres_sequence[i-1]);


                            // add circle fitting cost function
                            for(int j = 0;j < marker_points_parameter.size();j++){
                                auto &c = cluster[j];
                                auto& marker_ = marker_points_parameter[j];

                                if (c.empty()) continue;

                                ceres::CostFunction* cost_function = pose_solver::  MarkerFitCalib::create(c, marker_radius);
                                problem.AddResidualBlock(cost_function, nullptr, &(init_x[0]),&(marker_[0])  );


                            }
                            transform::Transform2d relative_move =  (std::get<0>(ceres_sequence[i-1])).inverse() * (std::get<0>(ceres_sequence[i]));
                            double constrains[3] = {relative_move.x(),relative_move.y(),relative_move.yaw()};
                            double base_to_laser[3] = {base_to_laser_tf.x(),base_to_laser_tf.y(),base_to_laser_tf.yaw()};

#if 1
                            ceres::CostFunction* cost_function = pose_solver::  PoseConstrains::create(constrains, base_to_laser,ceres_constrains_weight_x,ceres_constrains_weight_y,ceres_constrains_weight_yaw);
                            problem.AddResidualBlock(cost_function, nullptr, &(init_x_last[0]),&(init_x[0])  );
#endif

                        }





                    }else{
                        std::cout << "run localization function" << std::endl;

                        for(int i = 1 ;i < CERES_SEQUENCE_NUM ;i++){

                            auto& cluster = std::get<1>(ceres_sequence[i]);
                            auto& init_x = std::get<3>(ceres_sequence[i]);
                            auto& init_x_last = std::get<3>(ceres_sequence[i-1]);


                            // add circle fitting cost function
                            for(int j = 0;j < marker_points_parameter.size();j++){
                                auto &c = cluster[j];
                                auto& marker_ = marker_points_parameter[j];

                                if (c.empty()) continue;

                                ceres::CostFunction* cost_function = pose_solver::  MarkerFit::create(c, marker_radius,marker_,ceres_fit_weight);
                                problem.AddResidualBlock(cost_function, nullptr, &(init_x[0]) );


                            }


#if 1
                            transform::Transform2d relative_move =  (std::get<0>(ceres_sequence[i-1])).inverse() * (std::get<0>(ceres_sequence[i]));
                            double constrains[3] = {relative_move.x(),relative_move.y(),relative_move.yaw()};
                            double base_to_laser[3] = {base_to_laser_tf.x(),base_to_laser_tf.y(),base_to_laser_tf.yaw()};
                            ceres::CostFunction* cost_function = pose_solver::  PoseConstrains::create(constrains, base_to_laser,ceres_constrains_weight_x,ceres_constrains_weight_y,ceres_constrains_weight_yaw);
                            problem.AddResidualBlock(cost_function, nullptr, &(init_x_last[0]),&(init_x[0])  );
#endif

                        }



                    }
                    ceres::Solver::Summary summary;

                    Solve(ceres_solver_options, &problem, &summary);
                    std::cout << summary.BriefReport() << "\n";
                    std::cout << "final    summary.initial_cost : " <<    summary.initial_cost<< ",    summary.final_cost : " <<    summary.final_cost<< std::endl;

                    if(summary.final_cost >0.04){
                        ROS_ERROR("summary.final_cost : %f",summary.final_cost );
                    }

                    calib_ok = true;

                }



                for(int i = 1 ;i < CERES_SEQUENCE_NUM ;i++){

                    auto& cluster = std::get<1>(ceres_sequence[i]);
                    auto& init_x = std::get<3>(ceres_sequence[i]);
                    auto& marker_ = marker_points_parameter[i];
                    std::cout << "==== " << i << "\n";
                    std::cout << "solved init_x: [ " << init_x[0] << ", " << init_x[1] << ", " << init_x[2] << " ]\n";


                }
                for(int j = 0;j < marker_points_parameter.size();j++){
                    auto& marker_ = marker_points_parameter[j];
                    std::cout << "solved marker_: [ " << marker_[0] << ", " << marker_[1] << ", " << marker_[2] << " ]\n";
                }

                auto& init_x = std::get<3>(ceres_sequence.back());


                transform::Transform2d solved_pose_inv(init_x[0],init_x[1],init_x[2]);

                std::cout << "solved solved_pose_inv: " << solved_pose_inv << std::endl;

                std::cerr << "tf transform2D:\n" << transform2D << std::endl;
                transform::Transform2d solved_pose(init_x[0],init_x[1],init_x[2]);

                if(ceres_mode == CERES_Match_ScanToMarker){
                    solved_pose = solved_pose*base_to_laser_tf.inverse();
                }else{
                    solved_pose = solved_pose_inv.inverse()*base_to_laser_tf.inverse();

                }


                std::cout << "solved solved_pose: " << solved_pose << std::endl;

                initialpose_msg.pose.pose.position.x = solved_pose.x();
                initialpose_msg.pose.pose.position.y = solved_pose.y();

                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(solved_pose.yaw()), initialpose_msg.pose.pose.orientation );
                initialpose_msg.header.stamp =   msg->header.stamp;

                solved_pub.publish(initialpose_msg);


                auto& temp_odom_to_base_tf = std::get<0>(ceres_sequence.back());

                // map_odom_tf * odom_base_tf * base_laser_tf = map_laser_tf
                transform::Transform2d map_odom_tf = solved_pose  *temp_odom_to_base_tf.inverse();

                tf::Transform map_odom_transform;
                map_odom_transform.setOrigin( tf::Vector3(map_odom_tf.x(), map_odom_tf.y(), 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, map_odom_tf.yaw());
                map_odom_transform.setRotation(q);

                ros::Time transform_expiration = (msg->header.stamp +
                                                  transform_tolerance_);

                pub_tf_stamped = tf::StampedTransform (map_odom_transform,
                                                    transform_expiration,
                                                    fixed_frame, odom_frame);

                pub_tf_stamped.stamp_ = ros::Time::now() +
                                        transform_tolerance_;


                std::cout << "map_odom_tf: " << map_odom_tf << std::endl;
                std::cout << "base_to_laser_tf: " << base_to_laser_tf << std::endl;
                std::cout << "odom_to_base_tf: " << odom_to_base_tf << std::endl;



                map_odom_tf_computed = true;
#if 0
                if(tf_broadcast){

                    tf_br.sendTransform(pub_tf_stamped);
                }


#endif

            }

        }


    };
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, cb);
    nh_private.setParam(amcl_tf_broadcast,true);


    while (ros::ok()) {


        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
        ros::spinOnce();

        nh_private.getParam(control_command_on_param,control_command_on);
        nh_private.getParam(detect_mode_param,detect_mode);
        nh_private.getParam(solve_fail_time_param,solve_fail_time);

        if(control_command_on != control_command_on_last){
            map_odom_tf_computed = false;

            ceres_sequence.clear();
            last_solve_time = common::FromUnixNow();


            control_command_on_last = control_command_on;

            //amcl_tf_broadcast
            nh_private.setParam(amcl_tf_broadcast,true);

            if(control_command_on){

                std::cerr << "start detection" << std::endl;
                nh_private.setParam(control_status_param,1);

            }else{
                std::cerr << "stop detection" << std::endl;

                nh_private.setParam(control_status_param,0);
            }

        }

        if(control_command_on &&  map_odom_tf_computed &&(map_odom_tf_computed != map_odom_tf_computed_last)){
            nh_private.setParam(control_status_param,2);

            if(  (detect_mode == DETECT_MODE_FREE) and tf_broadcast){
                nh_private.setParam(amcl_tf_broadcast,false);

            }
            if((detect_mode == DETECT_MODE_FREE_INIT)){
                initialpose_pub.publish(initialpose_msg);
            }


            map_odom_tf_computed_last = map_odom_tf_computed;


        }

        if(control_command_on &&  (solve_fail_time > 0.0)  && !map_odom_tf_computed){
            double dur_ms = common::ToMillSeconds(common::FromUnixNow() - last_solve_time);

            if(dur_ms > solve_fail_time){
                nh_private.setParam(control_status_param,-1);
                control_command_on = false;

            }
        }

        double scan_delay =  common::ToMillSeconds(common::FromUnixNow() - last_scan_time);
        if(scan_delay > 1000.0){
            std::cout << "scan data delay for " << scan_delay << " ms" << std::endl;
            std::cout << "scan data may not be published" << std::endl;


            last_scan_time = common::FromUnixNow();

        }
        if(map_odom_tf_computed ){
            if(  (detect_mode == DETECT_MODE_FREE) and tf_broadcast){
                pub_tf_stamped.stamp_ = ros::Time::now() +
                                        transform_tolerance_;
                tf_br.sendTransform(pub_tf_stamped);
            }
        }
    }

    nh_private.setParam(amcl_tf_broadcast,true);

    if (calib_or_loc){
        if(calib_ok){
            nlohmann::json  calib_data = marker_points;


            std::cout << "calib ok, write to " << calib_file << " with data :\n" << calib_data.dump() << std::endl;

            std::fstream s(calib_file, s.binary | s.trunc | s.in | s.out);
            s << calib_data.dump() << std::endl;
            s.close();
        } else{
            std::cout << "calib fail " << std::endl;

        }

    }




}