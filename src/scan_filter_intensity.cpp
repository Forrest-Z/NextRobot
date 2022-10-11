//
// Created by waxz on 22-10-11.
//

#include <iostream>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "xmlrpcpp/XmlRpc.h"

#include <thread>


#include "transform/transform.h"
#include "sensor/laser_scan.h"





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


    std::vector<std::array<float,3>> marker_points;
    std::vector<float> marker_points_array_2d;
    std::vector<float> marker_points_array_2d_in_laser;
    std::vector<std::array<float,3>> marker_points_in_laser;

    std::array<float,3> pints;

    if (nh_private.hasParam(marker_param)){
        nh_private.getParam(marker_param,marker_points_in_map);
        nh_private.getParam(scan_filter_radius_param,scan_filter_radius);
        nh_private.getParam(min_intensity_param,min_intensity);


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
    marker_points_in_laser = marker_points;
    for(int i = 0; i < marker_points.size();i++){
        std::cout << marker_points[i][0] << ", " << marker_points[i][1] << ", " << marker_points[i][2] << std::endl;
        marker_points_array_2d[i+i] =  marker_points[i][0];
        marker_points_array_2d[i+i +1] =  marker_points[i][1];

    }



    //===
    std::string fixed_frame = "/map";

    tf::TransformListener tl_;

    tf::Transform transform_;
    tf::Transform identity_transform_;

    bool tf_transform_received = false;
    bool scan_received = false;

    tf::StampedTransform transform;

    ROS_INFO("Waiting for tf transform.");


    sensor::ScanToPoints scan_handler;

    scan_handler.filterOption.radius = scan_filter_radius;
    scan_handler.filterOption.min_intensity = min_intensity;

    transform::Transform2d transform2D;
    transform::Transform2d transform2D_inv;

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_filtered_intensity",10);
    sensor_msgs::LaserScan scan_msg;


    auto cb = [&]( const sensor_msgs::LaserScanConstPtr &msg) {

        scan_handler.getLocalPoints(msg->ranges,msg-> intensities, msg->angle_min,msg->angle_increment,msg->range_max   );


        // get time
        // lookup tf
        if(fixed_frame != msg->header.frame_id){
            try {
                tl_.waitForTransform(fixed_frame, msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
                tl_.lookupTransform(fixed_frame, msg->header.frame_id, ros::Time(0), transform);
                transform_.setOrigin(transform.getOrigin());
                transform_.setRotation(transform.getRotation());
                tf_transform_received = true;
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

            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
//            ros::Duration(1.0).sleep();
                tf_transform_received = false;
            }



        }else{
            tf_transform_received = true;
            scan_handler.getGlobalPoints();

        }


        // transform points

        // get json

        if (tf_transform_received) {

        }


    };
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, cb);


    while (ros::ok()) {


        using namespace std::chrono_literals;
        std::this_thread::sleep_for(20ms);
        ros::spinOnce();


    }

}