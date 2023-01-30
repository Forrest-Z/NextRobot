//
// Created by waxz on 22-11-14.
//

#include <thread>
#include <iostream>


#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"


#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarker.h>


#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "xmlrpcpp/XmlRpc.h"


#include "common/task.h"
#include "common/suspend.h"
#include "sensor/laser_scan.h"

#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/voxel_grid.h>


#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <pcl/registration/joint_icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/gicp.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>




#include <fstream>


#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
//#include "filesystem.hpp"
#include "ghc/filesystem.hpp"

namespace fs = ghc::filesystem;
#endif


#include "icp/Normal2dEstimation.h"


#include "nlohmann/json.hpp"
#include <plog/Log.h> // Step1: include the headers
#include "plog/Initializers/RollingFileInitializer.h"
#include "plog/Appenders/ColorConsoleAppender.h"


namespace serialization {
    bool to_json(nlohmann::json &data, const std::string &name, const transform::Transform2d &object);

    bool from_json(nlohmann::json &data, const std::string &name, transform::Transform2d &object);
}

#include "message/serialization_json.h"

namespace serialization {
    constexpr auto Transform2d_properties = std::make_tuple(
            property(&transform::Transform2d::matrix, "matrix")
    );
}

namespace serialization {
    bool to_json(nlohmann::json &data, const std::string &name, const transform::Transform2d &object) {
//     constexpr  auto properties = house_properties;
        return to_json_unpack(data, name, object, Transform2d_properties);
    }

    bool from_json(nlohmann::json &data, const std::string &name, transform::Transform2d &object) {

        return from_json_unpack(data, name, object, Transform2d_properties);

    }
}



void toEulerAngle(const float x, const float y, const float z, const float w, float &roll, float &pitch, float &yaw) {
// roll (x-axis rotation)
    float sinr_cosp = +2.0 * (w * x + y * z);
    float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    float sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    float siny_cosp = +2.0 * (w * z + x * y);
    float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
}

void to_yaw(const float x, const float y, const float z, const float w, float &yaw) {
// roll (x-axis rotation)
    float sinr_cosp = +2.0 * (w * x + y * z);
    float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);

// pitch (y-axis rotation)
    float sinp = +2.0 * (w * y - z * x);


// yaw (z-axis rotation)
    float siny_cosp = +2.0 * (w * z + x * y);
    float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
}


void yaw_to_quaternion(float yaw, double &qx, double &qy, double &qz, double &qw) {
    float roll = 0.0;
    float pitch = 0.0;

    qx = 0.0;
    qy = 0.0;
    qz = sin(yaw * 0.5f);
    qw = cos(yaw * 0.5f);

    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

}

namespace ros_tool {
    class InteractiveTf {
        ros::NodeHandle nh_;
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

        void processFeedback(unsigned ind, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

        visualization_msgs::InteractiveMarker int_marker_;

        geometry_msgs::Pose pose_;
        tf::TransformBroadcaster br_;
        std::string parent_frame_;
        std::string frame_;

        void updateTf(int, const ros::TimerEvent &event);

        ros::Timer tf_timer_;

        std::function<void(const geometry_msgs::Pose &)> m_callback;
        bool is_start = false;

    public:
        InteractiveTf(const std::string &t_parent_frame, const std::string &t_target_frame);

        ~InteractiveTf();

        void start(float scale = 1.0, bool pub_tf = false);

        void setInitPose(float x, float y, float yaw);

        geometry_msgs::Pose &getPose();

        void setCallBack(std::function<void(const geometry_msgs::Pose &)> &&);
    };


    void InteractiveTf::setCallBack(std::function<void(const geometry_msgs::Pose &)> &&t_cb) {
        m_callback = std::move(t_cb);
    }

    void InteractiveTf::setInitPose(float x, float y, float yaw) {
        pose_.position.x = x;
        pose_.position.y = y;
        pose_.position.z = 1.0;

        yaw_to_quaternion(yaw, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);

    }

    geometry_msgs::Pose &InteractiveTf::getPose() {
        return pose_;
    }

    InteractiveTf::InteractiveTf(const std::string &t_parent_frame, const std::string &t_target_frame) :
            parent_frame_(t_parent_frame),
            frame_(t_target_frame) {}

    void InteractiveTf::start(float scale, bool pub_tf) {

        if(is_start){
            std::cout << "InteractiveTf is already running" << std::endl;

            return;
        }

        server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_tf"));

        // TODO(lucasw) need way to get parameters out- tf echo would work

        int_marker_.header.frame_id = parent_frame_;
        // http://answers.ros.org/question/262866/interactive-marker-attached-to-a-moving-frame/
        // putting a timestamp on the marker makes it not appear
        // int_marker_.header.stamp = ros::Time::now();
        int_marker_.name = "interactive_tf";
        int_marker_.description = "control a tf with 6dof";
        int_marker_.pose = pose_;
        int_marker_.scale = 1.0;

        {
            visualization_msgs::InteractiveMarkerControl control;

            // TODO(lucasw) get roll pitch yaw and set as defaults

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "rotate_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//            int_marker_.controls.push_back(control);
            control.name = "move_x";
            // TODO(lucasw) how to set initial values?
            // double x = 0.0;
            // ros::param::get("~x", x);
            // control.pose.position.x = x;
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker_.controls.push_back(control);
            // control.pose.position.x = 0.0;

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//            int_marker_.controls.push_back(control);
            control.name = "move_y";
            // double y = 0.0;
            // control.pose.position.z = ros::param::get("~y", y);
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker_.controls.push_back(control);
            // control.pose.position.y = 0.0;

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker_.controls.push_back(control);
            control.name = "move_z";
            // double z = 0.0;
            // control.pose.position.z = ros::param::get("~z", z);
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//            int_marker_.controls.push_back(control);
            // control.pose.position.z = 0.0;


        }

        server_->insert(int_marker_);
        server_->setCallback(int_marker_.name,
                             boost::bind(&InteractiveTf::processFeedback, this, 0, boost::placeholders::_1));
        // server_->setCallback(int_marker_.name, testFeedback);

        server_->applyChanges();

        if (pub_tf) {
            tf_timer_ = nh_.createTimer(ros::Duration(0.05),
                                        boost::bind(&InteractiveTf::updateTf, this, 0, boost::placeholders::_1));
        }
        is_start = true;
        std::cout << "InteractiveTf start" << std::endl;
    }

    InteractiveTf::~InteractiveTf() {
        server_.reset();
    }

    void InteractiveTf::updateTf(int, const ros::TimerEvent &event) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose_.position.x, pose_.position.y, pose_.position.z));
        transform.setRotation(tf::Quaternion(pose_.orientation.x,
                                             pose_.orientation.y,
                                             pose_.orientation.z,
                                             pose_.orientation.w));
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                               parent_frame_, frame_));
    }

    void InteractiveTf::processFeedback(
            unsigned ind,
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        ROS_DEBUG_STREAM(feedback->header.frame_id);
        pose_ = feedback->pose;
        if (m_callback) {
            m_callback(pose_);
        }
        ROS_DEBUG_STREAM(feedback->control_name);
        ROS_DEBUG_STREAM(feedback->event_type);
        ROS_DEBUG_STREAM(feedback->mouse_point);
        // TODO(lucasw) all the pose changes get handled by the server elsewhere?
        server_->applyChanges();
    }
}


// LaserScan to PointCloud

void createPointCloud2(sensor_msgs::PointCloud2 &cloud, const std::vector<std::string> &filed) {
//    cloud.header.frame_id = "map";
    cloud.height = 1;
    int filed_num = filed.size();

    cloud.point_step = 4 * filed_num;
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    cloud.fields.resize(filed_num);

    for (int i = 0; i < filed_num; i++) {

        cloud.fields[i].name = filed[i];
        cloud.fields[i].offset = 4 * i;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }
#if 0
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[3].count = 1;
#endif

}

/*
 fill cloud filed [x,y]
 */
void LaserScanToPointCloud2(const std::vector<float> &scan_points, int point_num, sensor_msgs::PointCloud2 &cloud) {
    int point_num_ = 0.5 * (scan_points.size());
    if (point_num_ <= point_num) {
        return;
    }
    cloud.row_step = point_num * cloud.point_step;
    cloud.data.resize(cloud.row_step);
    cloud.width = point_num;
    for (int i = 0; i < point_num; i++) {
        uint8_t *data_pointer = &cloud.data[0] + i * cloud.point_step;
        *(float *) data_pointer = scan_points[i + i];
        data_pointer += 4;
        *(float *) data_pointer = scan_points[i + i + 1];
        data_pointer += 4;
        *(float *) data_pointer = 0.0;

    }
}

void LaserScanToPclPointCloud(const std::vector<float> &scan_points, int point_num,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {


//    PLOGD <<"point_num: " << point_num << std::endl;
    auto &cloud = *cloud_ptr;
    // Fill in the cloud data
    cloud.width = point_num;
    cloud.height = 1;
//    cloud.is_dense = true;
    cloud.resize(cloud.width * cloud.height, pcl::PointXYZ{0.0, 0.0, 0.0});

    for (int i = 0; i < point_num; i++) {
        cloud[i].x = scan_points[i + i];
        cloud[i].y = scan_points[i + i + 1];

    }

}

void LaserScanToPclPointCloud(const std::vector<float> &scan_points, int point_num,
                              pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr) {


    auto &cloud = *cloud_ptr;
    // Fill in the cloud data
    cloud.width = point_num;
    cloud.height = 1;
//    cloud.is_dense = true;
    cloud.resize(cloud.width * cloud.height, pcl::PointNormal{0.0, 0.0, 0.0});

    for (int i = 0; i < point_num; i++) {
        cloud[i].x = scan_points[i + i];
        cloud[i].y = scan_points[i + i + 1];

    }

}

// PointCloud to
void PclPointCloudToPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, sensor_msgs::PointCloud2 &cloud) {
    cloud.width = cloud_ptr->width;

    int point_num = cloud_ptr->size();

    cloud.row_step = point_num * cloud.point_step;
    cloud.data.resize(cloud.row_step);
    cloud.width = point_num;
    for (int i = 0; i < point_num; i++) {
        uint8_t *data_pointer = &cloud.data[0] + i * cloud.point_step;
        *(float *) data_pointer = cloud_ptr->at(i).x;
        data_pointer += 4;
        *(float *) data_pointer = cloud_ptr->at(i).y;
        data_pointer += 4;
        *(float *) data_pointer = 0.0;

    }


}

// PointCloud to
void PclPointCloudToPointCloud2(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr, sensor_msgs::PointCloud2 &cloud) {
    cloud.width = cloud_ptr->width;

    int point_num = cloud_ptr->size();

    cloud.row_step = point_num * cloud.point_step;
    cloud.data.resize(cloud.row_step);
    cloud.width = point_num;
    for (int i = 0; i < point_num; i++) {
        uint8_t *data_pointer = &cloud.data[0] + i * cloud.point_step;
        *(float *) data_pointer = cloud_ptr->at(i).x;
        data_pointer += 4;
        *(float *) data_pointer = cloud_ptr->at(i).y;
        data_pointer += 4;
        *(float *) data_pointer = 0.0;
    }
}

void PclPointCloudToPoseArray(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr, geometry_msgs::PoseArray &pose_array) {


    int point_num = cloud_ptr->size();

    pose_array.poses.resize(point_num);

    float yaw = 0.0;
//    std::cout << "\nshow all yaw:\n";
    for (int i = 0; i < point_num; i++) {
        pose_array.poses[i].position.x = cloud_ptr->points[i].x;
        pose_array.poses[i].position.y = cloud_ptr->points[i].y;
        pose_array.poses[i].position.z = cloud_ptr->points[i].z;

//        pose_array.poses[i].orientation.x = 0.0;
//        pose_array.poses[i].orientation.y = 0.0;
//        pose_array.poses[i].orientation.z = 0.0;
//        pose_array.poses[i].orientation.w = 1.0;


        yaw = atan2(cloud_ptr->points[i].normal_y, cloud_ptr->points[i].normal_x);
//        std::cout << "[" << yaw << ", " << cloud_ptr->points[i].normal_y << ", " << cloud_ptr->points[i].normal_x << "], ";
        yaw_to_quaternion(yaw, pose_array.poses[i].orientation.x, pose_array.poses[i].orientation.y,
                          pose_array.poses[i].orientation.z, pose_array.poses[i].orientation.w);

    }
//    std::cout << "\nend show all yaw\n";

}


struct PointsStamped {
    std::vector<float> points;
    ros::Time stamp;
};

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;


#include <fstream>

//https://stackoverflow.com/questions/2602013/read-whole-ascii-file-into-c-stdstring
bool loadFileToStr(const std::string &filename, std::string &data) {
    std::ifstream t(filename.c_str());

    if (!t.is_open()) {
        return false;
    }
    std::stringstream buffer;
    buffer << t.rdbuf();
    data = buffer.str();
    return true;
}

bool dumpStrToFile(const std::string &filename, const std::string &data) {
    std::ofstream t(filename.c_str());
    t << data;

    return true;

}


struct MovementCheck {
    float no_move_translation_epsilon = 5e-3;
    float no_move_rotation_epsilon = 5e-3;
    float final_no_move_translation_epsilon = 1e-2;
    float final_no_move_rotation_epsilon = 1e-2;

    float move_translation_epsilon = 0.1;
    float move_rotation_epsilon = 0.05;

    float no_move_check_ms = 200.0;
    transform::Transform2d last_pose_inv;
    transform::Transform2d last_move_pose_inv;

    transform::Transform2d start_check_pose_inv;

    transform::Transform2d movement;

    common::Time last_time;

    bool start_check = false;
    bool still = false;
    long move_flag = 0;
    long move_flag_last = 0;
    long update_flag = 0;

    void reset(){

        start_check = false;
        still = false;

        move_flag = 0;
        move_flag_last = 0;
        update_flag = 0;
    }
    // trigger at move or first frame
    bool checkMoveTrigger(const transform::Transform2d &new_pose) {
        movement = last_move_pose_inv * new_pose;

        bool is_move = std::abs(movement.x()) > move_translation_epsilon
                || std::abs(movement.y()) > move_translation_epsilon
                || std::abs(movement.yaw()) > move_rotation_epsilon;

        if(is_move|| (update_flag == 0)){
            last_move_pose_inv = new_pose.inverse();
            move_flag++;
        }

//        PLOGD << "movement " << movement << "\n move_flag " << move_flag << "\n move_flag_last " << move_flag_last << std::endl;

        update_flag++;
        return move_flag!=move_flag_last;

    }
    bool isMoveTriggered(){
        return move_flag!=move_flag_last;
    }
    bool checkStill(const transform::Transform2d &new_pose) {

        movement = last_pose_inv * new_pose;

//        PLOGD << "new_pose:\n" << new_pose << std::endl;
//        PLOGD << "movement:\n" << movement << std::endl;

        bool no_move = std::abs(movement.x()) < no_move_translation_epsilon &&
                       std::abs(movement.y()) < no_move_translation_epsilon
                       && std::abs(movement.yaw()) < no_move_rotation_epsilon;

        movement = start_check_pose_inv * new_pose;
//        PLOGD << "movement:\n" << movement << std::endl;

        no_move = no_move && std::abs(movement.x()) < final_no_move_translation_epsilon &&   std::abs(movement.y()) < final_no_move_translation_epsilon  && std::abs(movement.yaw()) < final_no_move_rotation_epsilon;

        if ((no_move && !start_check) || !no_move) {
            last_time = common::FromUnixNow();
            start_check_pose_inv = new_pose.inverse();
        }
        last_pose_inv = new_pose.inverse();

        start_check = no_move;
        still = common::ToMillSeconds(common::FromUnixNow() - last_time) > no_move_check_ms;


        return still;

    }

    bool isStill()  {
//        still = common::ToMillSeconds(common::FromUnixNow() - last_time) > no_move_check_ms;

        return still;
    }

    float getStillTime(){
        return common::ToMillSeconds(common::FromUnixNow() - last_time);
    }


};


namespace pcl{
    /*
             pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::search::KdTree<pcl::PointXYZ> kdtree_2;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

     */

    template<typename PointT>
    void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::KdTreeFLANN<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
    }

    template<typename PointT>
    void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::search::KdTree<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
    }

    template<typename PointT>
    void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::octree::OctreePointCloudSearch<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
        t_tree.addPointsFromInputCloud ();
    }


    template<typename PointT, typename TreeType>
    class Normal2dEstimation{

    public:
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;
        using Scalar = float;

    private:
        const TreeType& m_tree;
        float m_query_radius = 0.1;
        PointCloudConstPtr m_input_cloud;

        std::vector<int> m_query_indices;
        std::vector<float> m_query_distance;

    public:
        Normal2dEstimation(const TreeType& t_tree, float radius = 0.1, int num = 20):
                m_tree(t_tree),
                m_query_radius(radius),
                m_query_indices(num),
                m_query_distance(num){

        }

        inline void
        setInputCloud(const PointCloudConstPtr& t_input_cloud)
        {
            m_input_cloud = t_input_cloud;
        }
        void setRadius(){

        }
        void setTree(){

        }
        void setInputCloud(){

        }

        void compute( const pcl::PointCloud<pcl::PointNormal>::Ptr&  output, float viewpoint_x, float viewpoint_y){

            int input_point_num = m_input_cloud->points.size();
            output->points.resize(input_point_num,pcl::PointNormal(0.0,0.0,0.0,0.0,0.0,0.0));
            output->height = m_input_cloud->height;
            output->width = m_input_cloud->width;
            output->is_dense = true;

            int rt = 0;
            Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
            Eigen::Matrix<Scalar, 2, 1> K(0.0, 0.0);
            Eigen::Matrix<Scalar, 4, 1> centroid;
            Eigen::Matrix<Scalar, 2, 2> covariance_matrix;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig_solver(2);

            for(int i = 0 ; i < input_point_num;i++){


                auto& query_point = m_input_cloud->at(i);
                rt = m_tree.radiusSearch (query_point, m_query_radius, m_query_indices, m_query_distance);

                std::size_t point_count;
                point_count = m_query_indices.size ();
                if(point_count <=3){
                    output->points[i].normal_x  = output->points[i].normal_y  = output->points[i].normal_z  = output->points[i].curvature = std::numeric_limits<float>::quiet_NaN ();
                    continue;
                }


                K.x() = m_input_cloud->at(m_query_indices[0]).x;
                K.y() = m_input_cloud->at(m_query_indices[0]).y;


                for (const auto &index : m_query_indices)
                {
                    Scalar x = m_input_cloud->at(index).x - K.x(), y = m_input_cloud->at(index).y - K.y();
                    accu [0] += x * x;
                    accu [1] += x * y;
                    accu [2] += y * y;
                    accu [3] += x;
                    accu [4] += y;
                }


                {
                    accu /= static_cast<Scalar> (point_count);
                    centroid[0] = accu[3] + K.x(); centroid[1] = accu[4] + K.y(); centroid[2] = 0.0;
                    centroid[3] = 1;
                    covariance_matrix.coeffRef (0) = accu [0] - accu [3] * accu [3];//xx
                    covariance_matrix.coeffRef (1) = accu [1] - accu [3] * accu [4];//xy
                    covariance_matrix.coeffRef (3) = accu [2] - accu [4] * accu [4];//yy
                    covariance_matrix.coeffRef (2) = covariance_matrix.coeff (1);//yx



                    eig_solver.compute(covariance_matrix);
#if 0
                    {
                        Eigen::MatrixX2f m(m_query_indices.size(),2);
                        int index = 0;
                        for(int j :  m_query_indices){
                            m(index,0) = m_input_cloud->at(j).x ;
                            m(index,1) = m_input_cloud->at(j).y ;
                            index++;
                        }

                        Eigen::VectorXf mean_vector = m.colwise().mean();

                        Eigen::MatrixXf centered = m.rowwise() - mean_vector.transpose();

                        Eigen::MatrixXf cov = (centered.adjoint() * centered) / ( m.rows() - 1 ) ;
                        eig_solver.compute(cov);

                    }
#endif






                    auto& eigen_values = eig_solver.eigenvalues();
                    auto& eigen_vectors = eig_solver.eigenvectors() ;
                    auto& nx =  output->points[i].normal_x;
                    auto& ny =  output->points[i].normal_y;
                    auto& nz =  output->points[i].normal_z;
                    auto& c =  output->points[i].curvature;

                    nx = eigen_vectors(0,0);
                    ny = eigen_vectors(1,0);
#if 0
                    if(std::abs(eigen_values(0)) < std::abs(eigen_values(1))){
                        nx = eigen_vectors(0,0);
                        ny = eigen_vectors(1,0);
                    }else{
                        nx = eigen_vectors(0,1);
                        ny = eigen_vectors(1,1);
                    }
#endif
                    nz = 0.0;

                    Eigen::Matrix <float, 2, 1> normal (nx, ny);
                    Eigen::Matrix <float, 2, 1> vp (viewpoint_x - query_point.x, viewpoint_y - query_point.y);

                    // Dot product between the (viewpoint - point) and the plane normal
                    float cos_theta = vp.dot (normal);
                    // Flip the plane normal
                    if (cos_theta < 0)
                    {
                        nx *= -1;
                        ny *= -1;
                    }

                    c = std:: acos(std::abs(cos_theta));

//                    std::cout << __LINE__ << "eigenvalues:\n" << eigen_values << std::endl;
//                    std::cout << __LINE__ << "eigenvectors:\n" << eigen_vectors << std::endl;
                }


            }

        }

    };
}

template <typename T>
T normalise_angle(){

}

class MapOdomFilter{
private:
    int max_len = 5;
    std::deque<transform::Transform2d> buffer;

public:
    MapOdomFilter(int t_max_len = 5):max_len(t_max_len){ }
    void setLen(int t_max_len){
        max_len = t_max_len;
    }
    transform::Transform2d add(const transform::Transform2d& t_pose){
        if(buffer.size() > max_len){
            buffer.pop_back();
        }
        buffer.emplace_front(t_pose);

        float x = 0.0, y = 0.0 ,yaw = 0.0;
        transform::Transform2d relative_pose , first_pose_inv = t_pose.inverse();
        for(int i = 1 ; i < buffer.size();i++){
            relative_pose = first_pose_inv*buffer[i];

            x += relative_pose.x();
            y += relative_pose.y();
            yaw += relative_pose.yaw();
        }
        x /= buffer.size();
        y /= buffer.size();
        yaw /= buffer.size();

        relative_pose.set(x,y,yaw);

        return relative_pose;
    }
};


/*
use 2d/3d pointcloud to solve robot pose;

 main function
 1. filter input cloud
 2. match with reference cloud
 3. update map use input cloud and exist reference cloud


 details:
 1.1 voxel grid filter, radius filter, condition filter

 2.1 point to plane icp, weights

 3.1 use icp solved pose and cloud to update reference
 3.2 due to noise in icp solved pose, add every input cloud to reference cloud may lead to mess
 3.3 take movement into consideration, only use input cloud in standstill state


 */
class PointCloudPoseSolver {
public:
    enum class Mode {
        mapping,
        localization
    };

    struct FilterReadingConfig {
        float voxel_grid_size = 0.05;
        float radius_rmv_radius = 0.09;
        int radius_rmv_nn = 3;
        float cond_curve = 0.1;

    };

    struct FilterReferenceConfig {
        float voxel_grid_size = 0.1;
        float radius_rmv_radius = 0.2;
        int radius_rmv_nn = 4;
    };

    struct NormEstConfig {
        float radius = 0.06;
    };

    struct IcpConfig {

        float max_match_distance = 0.2;
        float ransac_outlier = 0.03;
        int max_iter = 20;
        float transformation_epsilon = 1e-4;
        float rotation_epsilon = 1e-4;


    };

    struct MoveCheckConfig {
        float no_move_translation_epsilon = 5e-3;
        float no_move_rotation_epsilon = 5e-3;
        float final_no_move_translation_epsilon = 1e-2;
        float final_no_move_rotation_epsilon = 1e-2;
        float no_move_check_ms = 200.0;

    };


    struct ErrorRejectionConfig{
        float max_translation_jump = 0.2;
        float max_rotation_jump = 0.2;
        float match_ratio = 0.2;

    };

    FilterReadingConfig &getFilterReadingConfig() {

        return m_filter_reading_config;
    }

    FilterReferenceConfig &getFilterReferenceConfig() {

        return m_filter_reference_config;
    }

    NormEstConfig &getNormEstConfig() {
        return m_norm_est_config;
    }

    IcpConfig &getIcpConfig() {
        return m_icp_config;
    }

    MoveCheckConfig &getMoveCheckConfig() {
        return m_move_check_config;
    }

    ErrorRejectionConfig& getErrorRejectionConfig(){
        return m_error_rejection_config;

    }
    using ICPTYPE = pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>;
//    using ICPTYPE = pcl::registration::TransformationEstimationLM<pcl::PointNormal, pcl::PointNormal>;


private:
    FilterReadingConfig m_filter_reading_config;
    FilterReferenceConfig m_filter_reference_config;
    NormEstConfig m_norm_est_config;
    IcpConfig m_icp_config;
    MoveCheckConfig m_move_check_config;
    ErrorRejectionConfig m_error_rejection_config;


    Mode mode = Mode::mapping;

    // cloud reference frame pose, in /map frame
    transform::Transform2d origin;
    // sensor pose in /base_link frame
    transform::Transform2d sensor_relative_pose;
    // sensor pose in /origin frame
    transform::Transform2d sensor_absolute_pose;
    transform::Transform2d sensor_absolute_pose_last;
    transform::Transform2d sensor_absolute_pose_change;

    // robot pose in origin frame
    transform::Transform2d robot_relative_pose;
    // robot pose in /map frame
    transform::Transform2d robot_absolute_pose;

    // odom to base pose
    transform::Transform2d robot_odom_pose;

    // map to odom
    transform::Transform2d map_odom_pose;


    bool is_first_cloud = true;
    // use fist robot pose to compute origin
    bool is_origin_computed = false;

    bool is_sensor_pose_set = false;

    bool is_icp_init = true;

    bool is_icp_fault = false;

    bool is_icp_converged = false;


    // cloud as map
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_reference;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reading_filtered;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_reading_filtered_norm;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_align;
    pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud_norm;

    pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud_norm_2;

    // filter reading
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> m_radius_outlier_removal;
    pcl::VoxelGrid<pcl::PointXYZ> m_approximate_voxel_grid;
    pcl::ConditionAnd<pcl::PointNormal>::Ptr m_view_cond;

    pcl::ConditionalRemoval<pcl::PointNormal> m_cond_rem;


    // filter reference
    pcl::RadiusOutlierRemoval<pcl::PointNormal> m_refe_radius_outlier_removal;
    pcl::VoxelGrid<pcl::PointNormal> m_refe_approximate_voxel_grid;


    // icp
    pcl::registration::WarpPointRigid3D<pcl::PointNormal, pcl::PointNormal>::Ptr m_warp_fcn_pl;

    // test diffrent icp interface
    // 1. IterativeClosestPointWithNormals , original code, test ok
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> m_pl_icp;
//    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> m_pl_icp;
//    pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> m_pl_icp;

    pcl::JointIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> m_joint_pl_icp;


//    pcl::registration::TransformationEstimationPointToPlaneWeighted<pcl::PointNormal, pcl::PointNormal>::Ptr m_pl_te;
    ICPTYPE::Ptr m_pl_te;
    std::vector<double> m_pl_te_weights;

    // norm est
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_norm_tree;
    pcl::search::KdTree<pcl::PointNormal>::Ptr m_surface_tree;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> m_mls;

    pcl::octree::OctreePointCloudSearch<pcl::PointNormal> m_octree_reading ;
    pcl::octree::OctreePointCloudSearch<pcl::PointNormal> m_octree_reference ;
    pcl::Normal2dEstimation<pcl::PointNormal, pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> m_norm_est_reading;
    pcl::Normal2dEstimation<pcl::PointNormal, pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> m_norm_est_reference;


    Normal2dEstimation m_norm_estim;

    MovementCheck m_movement_check;

    MapOdomFilter map_odom_filter;

public:
    PointCloudPoseSolver() : cloud_reference(new pcl::PointCloud<pcl::PointNormal>),
                             temp_cloud_1(new pcl::PointCloud<pcl::PointXYZ>),
                             temp_cloud_2(new pcl::PointCloud<pcl::PointXYZ>),

                             cloud_reading_filtered(new pcl::PointCloud<pcl::PointXYZ>),
                             cloud_reading_filtered_norm(new pcl::PointCloud<pcl::PointNormal>),
                             cloud_align(new pcl::PointCloud<pcl::PointNormal>),
                             temp_cloud_norm(new pcl::PointCloud<pcl::PointNormal>),
                             temp_cloud_norm_2(new pcl::PointCloud<pcl::PointNormal>),

                             m_view_cond(new pcl::ConditionAnd<pcl::PointNormal>()),

                             m_warp_fcn_pl
                                     (new pcl::registration::WarpPointRigid3D<pcl::PointNormal, pcl::PointNormal>),
                             m_norm_tree(new pcl::search::KdTree<pcl::PointXYZ>),
                             m_surface_tree(new pcl::search::KdTree<pcl::PointNormal>),
                             m_pl_te(
                                     new ICPTYPE) ,
                             m_pl_te_weights(1000),
                             m_octree_reading(0.05),
                             m_octree_reference(0.05),m_norm_est_reading(m_octree_reading),m_norm_est_reference(m_octree_reference){


        config();

    }


    // config filter, norm_est, icp
    void config() {

        m_approximate_voxel_grid.setLeafSize(m_filter_reading_config.voxel_grid_size,
                                             m_filter_reading_config.voxel_grid_size,
                                             m_filter_reading_config.voxel_grid_size);
        m_radius_outlier_removal.setRadiusSearch(m_filter_reading_config.radius_rmv_radius);
        m_radius_outlier_removal.setMinNeighborsInRadius(m_filter_reading_config.radius_rmv_nn);


        m_view_cond.reset(new pcl::ConditionAnd<pcl::PointNormal>());
        m_view_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
                new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::LT,
                                                           m_filter_reading_config.cond_curve)));
        m_cond_rem.setCondition(m_view_cond);

        //        m_cond_rem.setKeepOrganized(true);


        m_refe_approximate_voxel_grid.setLeafSize(m_filter_reference_config.voxel_grid_size,
                                                  m_filter_reference_config.voxel_grid_size,
                                                  m_filter_reference_config.voxel_grid_size);
        m_refe_radius_outlier_removal.setRadiusSearch(m_filter_reference_config.radius_rmv_radius);
        m_refe_radius_outlier_removal.setMinNeighborsInRadius(m_filter_reference_config.radius_rmv_nn);


        m_norm_estim.setSearchMethod(m_norm_tree);
        m_norm_estim.setRadiusSearch(m_norm_est_config.radius);

        m_pl_te->setWarpFunction(m_warp_fcn_pl);


        m_pl_icp.setTransformationEstimation(m_pl_te);

        m_pl_icp.setMaxCorrespondenceDistance(m_icp_config.max_match_distance);

        m_pl_icp.setRANSACOutlierRejectionThreshold(m_icp_config.ransac_outlier);

        m_pl_icp.setMaximumIterations(m_icp_config.max_iter);
        m_pl_icp.setTransformationEpsilon(m_icp_config.transformation_epsilon);
        m_pl_icp.setEuclideanFitnessEpsilon(m_icp_config.rotation_epsilon);
        m_pl_icp.setTransformationRotationEpsilon(m_icp_config.rotation_epsilon);

#if 0
        m_pl_icp.setRANSACOutlierRejectionThreshold(0.08);
        m_pl_icp.setRANSACIterations(4);
#endif

        m_joint_pl_icp.setTransformationEstimation(m_pl_te);

        m_movement_check.no_move_check_ms = m_move_check_config.no_move_check_ms;
        m_movement_check.no_move_translation_epsilon = m_move_check_config.no_move_translation_epsilon;
        m_movement_check.no_move_rotation_epsilon = m_move_check_config.no_move_rotation_epsilon;
        m_movement_check.final_no_move_translation_epsilon = m_move_check_config.final_no_move_translation_epsilon;
        m_movement_check.final_no_move_rotation_epsilon = m_move_check_config.final_no_move_rotation_epsilon;


        m_mls.setComputeNormals (true);

        // Set parameters
        m_mls.setPolynomialOrder (1);
        m_mls.setSearchMethod (m_surface_tree);
        m_mls.setSearchRadius (0.2);

        m_mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal>::NONE);
        m_mls.setUpsamplingRadius (0.1);
        m_mls.setUpsamplingStepSize (0.1);



        map_odom_filter.setLen(20);

    }


    pcl::PointCloud<pcl::PointNormal>::Ptr getCloudReading() const {
        return cloud_reading_filtered_norm;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr getCloudMatched() const {
        return cloud_align;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr getCloudReference() const {
        return cloud_reference;
    }

    void setMode(const Mode &m) {
        mode = m;
    }

    // voxel_grrid_filter
    // radius_remove_filter
    void
    filterReading(pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud_filtered) {
        m_approximate_voxel_grid.setInputCloud(t_cloud);
        m_approximate_voxel_grid.filter(*temp_cloud_1);

        m_radius_outlier_removal.setInputCloud(temp_cloud_1);
        m_radius_outlier_removal.filter(*t_cloud_filtered);
    }


    float icp_match(pcl::PointCloud<pcl::PointNormal>::Ptr t_reading, pcl::PointCloud<pcl::PointNormal>::Ptr t_cloud,
                    pcl::PointCloud<pcl::PointNormal>::Ptr t_align,
                    transform::Transform2d &est_pose) {


        Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity ());
        initial_guess(0, 0) = est_pose.matrix[0][0];
        initial_guess(0, 1) = est_pose.matrix[0][1];
        initial_guess(1, 0) = est_pose.matrix[1][0];
        initial_guess(1, 1) = est_pose.matrix[1][1];
        initial_guess(0, 3) = est_pose.matrix[0][2];
        initial_guess(1, 3) = est_pose.matrix[1][2];
#if 0
        m_pl_te_weights.resize(t_reading->size());
        for(int i = 0; i < m_pl_te_weights.size();i++){
            m_pl_te_weights[i] = 1.0 - std::abs(t_reading->points[i].curvature);
        }
        m_pl_te->setWeights(m_pl_te_weights);
#endif
//        PLOGD << "t_reading size = " << t_reading->size() << std::endl;
        m_pl_icp.setInputSource(t_reading);
        m_pl_icp.setInputTarget(t_cloud);
        m_pl_icp.align(*t_align, initial_guess);


        Eigen::Matrix4f result_pose = m_pl_icp.getFinalTransformation();

        est_pose.matrix[0][0] = result_pose(0, 0);
        est_pose.matrix[0][1] = result_pose(0, 1);
        est_pose.matrix[1][0] = result_pose(1, 0);
        est_pose.matrix[1][1] = result_pose(1, 1);
        est_pose.matrix[0][2] = result_pose(0, 3);
        est_pose.matrix[1][2] = result_pose(1, 3);

        est_pose.matrix[2][0] = 0.0;
        est_pose.matrix[2][1] = 0.0;
        est_pose.matrix[2][2] = 1.0;

//        std::cout << "m_pl_icp has converged:" << m_pl_icp.hasConverged() << " score: " << m_pl_icp.getFitnessScore() << std::endl;
//        std::cout << "getFinalTransformation:\n" << m_pl_icp.getFinalTransformation() << "\n";

//        PLOGD << "m_pl_icp getFinalTransformation:\n" << m_pl_icp.getFinalTransformation() << std::endl;

        return m_pl_icp.getFitnessScore();

    }

    float icp_match_v2(pcl::PointCloud<pcl::PointNormal>::Ptr t_reading, pcl::PointCloud<pcl::PointNormal>::Ptr t_cloud,
                    pcl::PointCloud<pcl::PointNormal>::Ptr t_align,
                    transform::Transform2d &est_pose) {


        Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity ());
        initial_guess(0, 0) = est_pose.matrix[0][0];
        initial_guess(0, 1) = est_pose.matrix[0][1];
        initial_guess(1, 0) = est_pose.matrix[1][0];
        initial_guess(1, 1) = est_pose.matrix[1][1];
        initial_guess(0, 3) = est_pose.matrix[0][2];
        initial_guess(1, 3) = est_pose.matrix[1][2];
#if 0
        m_pl_te_weights.resize(t_reading->size());
        for(int i = 0; i < m_pl_te_weights.size();i++){
            m_pl_te_weights[i] = 1.0 - std::abs(t_reading->points[i].curvature);
        }
        m_pl_te->setWeights(m_pl_te_weights);
#endif
        m_joint_pl_icp.addInputSource(t_reading);
        m_joint_pl_icp.addInputTarget(t_cloud);
        m_joint_pl_icp.align(*t_align, initial_guess);


        Eigen::Matrix4f result_pose = m_joint_pl_icp.getFinalTransformation();

        est_pose.matrix[0][0] = result_pose(0, 0);
        est_pose.matrix[0][1] = result_pose(0, 1);
        est_pose.matrix[1][0] = result_pose(1, 0);
        est_pose.matrix[1][1] = result_pose(1, 1);
        est_pose.matrix[0][2] = result_pose(0, 3);
        est_pose.matrix[1][2] = result_pose(1, 3);

        est_pose.matrix[2][0] = 0.0;
        est_pose.matrix[2][1] = 0.0;
        est_pose.matrix[2][2] = 1.0;

//        std::cout << "m_joint_pl_icp has converged:" << m_joint_pl_icp.hasConverged() << " score: " << m_joint_pl_icp.getFitnessScore() << std::endl;
//        std::cout << "getFinalTransformation:\n" << m_joint_pl_icp.getFinalTransformation() << "\n";
//        PLOGD << "m_joint_pl_icp getFinalTransformation:\n" << m_joint_pl_icp.getFinalTransformation() << std::endl;

        return m_joint_pl_icp.getFitnessScore();

    }

    // update reference with aligned pointcloud
    // filter
    void updateReference(const pcl::PointCloud<pcl::PointNormal>::Ptr t_reading) {

//        PLOGD << "t_reading->size() " << t_reading->size() << std::endl;
        if (cloud_reference->size() == 0) {
            (*cloud_reference) += (*t_reading);
            return;
        }

#if 0

#endif

        m_surface_tree->setInputCloud(cloud_reference);

        pcl::ExtractIndices<pcl::PointNormal> extract;



        pcl::PointNormal searchPoint;
        float radius = 0.2;

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        int point_num = t_reading->size();

        std::set<int> removed_idx,removed_idx_input;

        for(int i = 0; i < point_num;i++){
            searchPoint = t_reading->at(i);


            if ( m_surface_tree->radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {

                std::sort(pointRadiusSquaredDistance.begin(),pointRadiusSquaredDistance.end());
                if(pointRadiusSquaredDistance.front() > 0.005){
                    removed_idx.insert(pointIdxRadiusSearch.begin(),pointIdxRadiusSearch.end());
                    removed_idx_input.insert(i);

                }

            }

        }

        pcl::IndicesPtr indices (new pcl::Indices (removed_idx.size()));
        pcl::IndicesPtr indices_input (new pcl::Indices (removed_idx_input.size()));

//        PLOGD << "remove indices: " << removed_idx.size() << std::endl;
//        PLOGD << "remove indices_input: " << removed_idx_input.size() << std::endl;

        int i = 0;
        for(auto e:removed_idx){
            (*indices)[i]= e;
            i++;
        }
        i = 0;
        for(auto e:removed_idx_input){
            (*indices_input)[i]= e;
            i++;
        }

        extract.setInputCloud(cloud_reference);
        extract.setIndices(indices);
        extract.setNegative(true);
        extract.filter(*temp_cloud_norm);

        extract.setInputCloud(t_reading);
        extract.setIndices(indices_input);
        extract.setNegative(true);
        extract.filter(*temp_cloud_norm_2);


        (*cloud_reference) = (*temp_cloud_norm) + (*temp_cloud_norm_2);


        // using 1220
#if 1
        m_refe_approximate_voxel_grid.setInputCloud(cloud_reference);
        m_refe_approximate_voxel_grid.filter(*temp_cloud_norm);

        m_refe_radius_outlier_removal.setInputCloud(temp_cloud_norm);
        m_refe_radius_outlier_removal.filter(*cloud_reference);
#endif

#if 0
        m_mls.setInputCloud (cloud_reference);
        m_mls.process (*temp_cloud_norm);
        m_refe_approximate_voxel_grid.setInputCloud(temp_cloud_norm);
        m_refe_approximate_voxel_grid.filter(*cloud_reference);
#endif


#if 0

        pcl::copyPointCloud(*cloud_reference, *temp_cloud_1);

        m_refe_approximate_voxel_grid.setInputCloud(temp_cloud_1);
        m_refe_approximate_voxel_grid.filter(*temp_cloud_2);

        m_refe_radius_outlier_removal.setInputCloud(temp_cloud_2);
        m_refe_radius_outlier_removal.filter(*temp_cloud_1);

        computeNorm(temp_cloud_1, cloud_reference,
                    pcl::PointXYZ(sensor_absolute_pose.x(), sensor_absolute_pose.y(), 0.0));
#endif



#if 0
        pcl::copyPointCloud(*cloud_reference, *temp_cloud_1);

        m_approximate_voxel_grid.setInputCloud(temp_cloud_1);
        m_approximate_voxel_grid.filter(*cloud_reading_filtered);

        m_radius_outlier_removal.setInputCloud(cloud_reading_filtered);
        m_radius_outlier_removal.filter(*temp_cloud_1);
        computeNorm(temp_cloud_1, cloud_reference);

#endif

    }

    // compute norm
    void computeNorm(pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr t_cloud_norm,
                     const pcl::PointXYZ &view_point = pcl::PointXYZ(0.0, 0.0, 0.0)) {

        pcl::copyPointCloud(*t_cloud, *t_cloud_norm);
        m_norm_estim.setInputCloud(t_cloud);
        m_norm_estim.compute(t_cloud_norm);
        m_norm_estim.setViewPoint(view_point);
    }

    void removeReadingShadow(pcl::PointCloud<pcl::PointNormal>::Ptr t_cloud,
                             pcl::PointCloud<pcl::PointNormal>::Ptr t_cloud_norm) {

        m_cond_rem.setInputCloud(t_cloud);
        m_cond_rem.filter(*t_cloud_norm);
    }


    void matchUpdate( ) {

        updateReference(cloud_align);


    }

    void matchUpdate(const transform::Transform2d &odom_pose) {


//        std::cout << "check time: " << m_movement_check.getStillTime() << "\n";

        bool update = m_movement_check.checkStill(odom_pose) ;
//        std::cout << __FILE__ << ":" << __LINE__ << " , update : " << update << ", size : " << cloud_reference->size()   << "\n";


        if ((update ) || cloud_reference->size() == 0) {

//            std::cout << __FILE__ << ":" << __LINE__ << " call updateReference \n";

            updateReference(cloud_align);

        }

    }

    void match() {

        if (!cloud_reference->empty()) {
            if (!m_movement_check.isStill()) {
//                std::cout << " perform icp\n";
                float score = icp_match(cloud_reading_filtered_norm, cloud_reference, cloud_align,
                                        sensor_absolute_pose);
            } else {
//                std::cout << " bypass icp\n";

                Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity ());
                initial_guess(0, 0) = sensor_absolute_pose.matrix[0][0];
                initial_guess(0, 1) = sensor_absolute_pose.matrix[0][1];
                initial_guess(1, 0) = sensor_absolute_pose.matrix[1][0];
                initial_guess(1, 1) = sensor_absolute_pose.matrix[1][1];
                initial_guess(0, 3) = sensor_absolute_pose.matrix[0][2];
                initial_guess(1, 3) = sensor_absolute_pose.matrix[1][2];
                pcl::transformPointCloud(*cloud_reading_filtered_norm, *cloud_align, initial_guess);

            }

        } else {
            (*cloud_reference) += (*cloud_reading_filtered_norm);

        }

        robot_relative_pose = sensor_absolute_pose * sensor_relative_pose.inverse();
        robot_absolute_pose = origin * robot_relative_pose;


    }

    bool isIcpInit() const{
        return is_icp_init;
    }
    bool isIcpFault() const{
        return is_icp_fault;
    }
    bool isIcpWorking(){
        return (!cloud_reading_filtered_norm->empty()) && (!cloud_reference->empty()) && (!cloud_align->empty());
    }
    void match_v2() {


//        PLOGD << "try icp" << is_icp_init << ", " << is_icp_fault << std::endl;

        if(!is_icp_init || is_icp_fault){

            PLOGD << "skip icp" << is_icp_init << ", " << is_icp_fault << std::endl;

            return;
        }

        if(cloud_reading_filtered_norm->empty()){

            PLOGD << "cloud_reading_filtered_norm is empty" << std::endl;
            return;

        }

        if (!cloud_reference->empty()) {

//            PLOGD << "sensor_absolute_pose before icp : " << sensor_absolute_pose << std::endl;
            sensor_absolute_pose_last = sensor_absolute_pose;
//            PLOGD << "cloud_reading_filtered_norm.size: " << cloud_reading_filtered_norm->size() << ", cloud_reference.size: " << cloud_reference->size() << std::endl;


//            icp_match_v2(cloud_reading_filtered_norm, cloud_reference, cloud_align, sensor_absolute_pose);
            float score = icp_match(cloud_reading_filtered_norm, cloud_reference, cloud_align, sensor_absolute_pose);
//            PLOGD << "sensor_absolute_pose after  icp : " << sensor_absolute_pose << std::endl;
//            PLOGD << "sensor_absolute_pose sensor_absolute_pose_last: " << sensor_absolute_pose_last << std::endl;
//            PLOGD << "icp score : " << score << ", " << m_pl_icp.hasConverged()   << std::endl;


            sensor_absolute_pose_change = sensor_absolute_pose_last.inverse() *sensor_absolute_pose;


            is_icp_converged = !(!m_pl_icp.hasConverged()
                               || (std::abs(sensor_absolute_pose_change.x()) > m_error_rejection_config.max_translation_jump)
                               ||  (std::abs(sensor_absolute_pose_change.y()) > m_error_rejection_config.max_translation_jump)
                               || (std::abs(sensor_absolute_pose_change.yaw()) > m_error_rejection_config.max_rotation_jump));

            if(is_icp_converged){

            }
            else{
//                std::cout << "***************\nsensor_absolute_pose_change : fault\n *************************************";
//                PLOGD << "icp fault wait reset " << std::endl;
//                cloud_align->clear();


                   //check align
                m_surface_tree->setInputCloud(cloud_reference);

                pcl::PointNormal searchPoint;
                float radius = 0.2;

                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                int point_num = cloud_align->size();

//                std::set<int> removed_idx,removed_idx_input;

                int match_num = 0;
                for(int i = 0; i < point_num;i++){
                    searchPoint = cloud_align->at(i);


                    if ( m_surface_tree->radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
                    {

                        std::sort(pointRadiusSquaredDistance.begin(),pointRadiusSquaredDistance.end());
                        if(pointRadiusSquaredDistance.front() < 0.005){
                            match_num++;
//                            removed_idx.insert(pointIdxRadiusSearch.begin(),pointIdxRadiusSearch.end());
//                            removed_idx_input.insert(i);

                        }

                    }

                }

                float match_ratio_reading = float(match_num)/float(point_num);
                float match_ratio_reference = float(match_num)/float(cloud_reference->size());


                if( (match_ratio_reading < m_error_rejection_config.match_ratio) || (match_ratio_reference < m_error_rejection_config.match_ratio) ){

                    is_icp_fault = true;
                    sensor_absolute_pose = sensor_absolute_pose_last;

                    PLOGD << "icp sensor_absolute_pose_change : " << sensor_absolute_pose_change  << std::endl;

                    PLOGD <<"match_num: "<< match_num << ", point_num: " << point_num << ", match_ratio_reading: " << match_ratio_reading << ", match_ratio_reference: "<< match_ratio_reference<< std::endl;

                    PLOGD << "icp fault will reset " << std::endl;

                }else{

                }


            }




        } else {
            (*cloud_reference) += (*cloud_reading_filtered_norm);

        }


//        if(m_movement_check.getStillTime() < 100.0 )
        {

            robot_relative_pose = sensor_absolute_pose * sensor_relative_pose.inverse();
            robot_absolute_pose = origin * robot_relative_pose;

        }
    }

    bool matchReading() {

        if (cloud_reference->size() == 0) {
            //updateReference(cloud_reading_filtered_norm);
            (*cloud_reference) += (*cloud_reading_filtered_norm);

            return true;
        }

        float score = icp_match(cloud_reading_filtered_norm, cloud_reference, cloud_align, sensor_absolute_pose);

//        std::cout << "sensor_absolute_pose:\n" << sensor_absolute_pose << std::endl;
#if 0
        updateReference(cloud_align);
#endif

        return true;
    }


    // in loc mode
    // cloud_reference should be loadded from file
    int loadFromFile(const std::string & file_dir, const std::string& stamp) {

        fs::path output_dir(file_dir);
        fs::path output_stamp_dir = output_dir / stamp;

        fs::path path_pose = output_stamp_dir / "origin.json";
        fs::path path_pcd = output_stamp_dir / "cloud.pcd";

        if(! (fs::exists(path_pose)  && fs::exists(path_pcd) ) ){
            std::cerr << "solver load file error, " << path_pose.c_str() << ", " << path_pcd.c_str() << " not found"<< std::endl;

            return -1;
        }

        if (pcl::io::loadPCDFile<pcl::PointNormal> (path_pcd.c_str(), *cloud_reference) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s \n",path_pcd.c_str());
            return (-1);
        }

        std::string raw_data;
        loadFileToStr(path_pose,raw_data);
        nlohmann::json json_data = nlohmann::json::parse(raw_data);


        bool rt = serialization::from_json(json_data,"origin",origin);
        if(!rt){
            std::cout << "prase error: raw_data:\n" << raw_data << std::endl;

            return -1;
        }

        cloud_align->clear();
        is_origin_computed = true;
        is_icp_init = false;
        PLOGD << "resetIcp" << std::endl;
        resetIcp();


        std::cout << "load from file ok ; origin:\n" << origin << "\ncloud_reference size :  " << cloud_reference->size() << std::endl;


        return 0;
    }

    // in mapping mode
    // cloud_reference should be dumped to fle
    int dumpToFile(const std::string &file_dir,const std::string& stamp) {


        fs::path output_dir(file_dir);

        if (!fs::exists(output_dir)) {
            fs::create_directories(output_dir);
        }
        fs::path output_stamp_dir = output_dir / stamp;

        if (!fs::exists(output_stamp_dir)) {
            fs::create_directories(output_stamp_dir);
        }


        if(cloud_reference->empty()){
            std::cerr << "cloud_reference is empty"  << std::endl;

            return -1;

        }

        fs::path path_pose = output_stamp_dir / "origin.json";
        fs::path path_pcd = output_stamp_dir / "cloud.pcd";


        nlohmann::json json_data;
        serialization::to_json(json_data,"origin",origin);

        dumpStrToFile(path_pose,json_data.dump() );

        pcl::io::savePCDFileASCII(path_pcd.c_str(), *cloud_reference);
        std::cerr << "Saved " << cloud_reference->size() << " data points to " << path_pcd.c_str() << std::endl;

        return 0;
    }



    // add pointcloud with   movement assumption
    void addSensorReading(pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud_reading,
                          const transform::Transform2d &movement = transform::Transform2d()) {

        // input_cloud ->filter -> filtered_input_cloud
        filterReading(t_cloud_reading, cloud_reading_filtered);
        if(cloud_reading_filtered->empty()){
            return;
        }

        //origin
#if 1
        // filtered_input_cloud + norm for icp
        computeNorm(cloud_reading_filtered, temp_cloud_norm);

        //
        removeReadingShadow(temp_cloud_norm, cloud_reading_filtered_norm);
#endif

#if 0
        // filtered_input_cloud + norm for icp
        computeNorm(cloud_reading_filtered, cloud_reading_filtered_norm);

        //
//        removeReadingShadow(temp_cloud_norm, cloud_reading_filtered_norm);
#endif


#if 0
        // filtered_input_cloud + norm for icp
        computeNorm(cloud_reading_filtered, cloud_reading_filtered_norm);

        //
//        removeReadingShandow(temp_cloud_norm, cloud_reading_filtered_norm);
#endif


    }

    // set origin pose directly
    void setOrigin(const transform::Transform2d &t) {
        origin = t;
        is_origin_computed = true;
        is_icp_init = true;

    }

    const transform::Transform2d &getOrigin() {
        return origin;
    }

    void setSensorRelativePose(const transform::Transform2d &t) {
        sensor_relative_pose = t;
        is_sensor_pose_set = true;
    }

    bool isOriginCompute(){
        return is_origin_computed;
    }

    void setRobotAbsolutePoseOrigin(const transform::Transform2d &t) {
//        PLOGD<< "is_sensor_pose_set " << is_sensor_pose_set << " is_origin_computed " << is_origin_computed << " is_icp_init " << is_icp_init <<std::endl;
        if (is_sensor_pose_set && !is_origin_computed) {
            robot_absolute_pose = t;
            origin = robot_absolute_pose * robot_relative_pose.inverse();

            is_origin_computed = true;

        }
    }

    void setRobotAbsolutePoseInitIcp(const transform::Transform2d &t) {
//        PLOGD<< "is_sensor_pose_set " << is_sensor_pose_set << " is_origin_computed " << is_origin_computed << " is_icp_init " << is_icp_init <<std::endl;

        if (is_sensor_pose_set &&  is_origin_computed && !is_icp_init) {
            robot_absolute_pose = t;
            robot_relative_pose = origin.inverse() * robot_absolute_pose;

            sensor_absolute_pose = robot_relative_pose *sensor_relative_pose;
            sensor_absolute_pose_last = sensor_absolute_pose;

            is_icp_init = true;

        }
    }

    void resetIcp(){
//        PLOGD << "resetIcp" << std::endl;

        is_icp_init = false;
        is_icp_fault = false;
    }

    const transform::Transform2d &getRobotAbsolutePose() {
        return robot_absolute_pose;
    }

    const transform::Transform2d &getRobotRelativePose() {
        return robot_relative_pose;
    }

    const transform::Transform2d &solveMapOdom(const transform::Transform2d &t_odom_base) {



#if 0
        bool update = m_movement_check.checkStill(t_odom_base) ;
        std::cout << "solveMapOdom m_movement_check " << m_movement_check.isStill() << ", " << m_movement_check.getStillTime() <<  "\n";
        if(m_movement_check.isStill()){

            return map_odom_pose;
        }
#endif
//        robot_relative_pose = sensor_absolute_pose * sensor_relative_pose.inverse();
        robot_absolute_pose = origin * robot_relative_pose;
        map_odom_pose = robot_absolute_pose * t_odom_base.inverse();


#if 0
        map_odom_pose = map_odom_filter.add(map_odom_pose);
#endif
        return map_odom_pose;
    }

};

/*  todo
final target
1. remove dynamic object
2. filter noise

method
1. sensor data fusion window

*/





/*
 RealtimeScanBuilder

 accumulate scan data , create a filtered pointcloud

 1. robot dosen't move
 2. robot move
 */




/*

 filter LaserScan ranges
 0. create buffer
 1. compute mean and stddev for each beam
 2.

 */

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

//        PLOGD << "std_dev " << std_dev << std::endl;

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


struct MissionManager{
    int run_command = 0;
    int start_run = 0;
    std::function<void()> start_command_callback;
    std::function<void()> stop_command_callback;
    std::function<void()> start_run_callback;
    std::function<void()> stop_run_callback;

    void command(int t_run){

        if(t_run && ! run_command){
            run_command = t_run;
            start_run = 1;
            if(start_command_callback){
                start_command_callback();
            }
        }

        if(!t_run && run_command){
            run_command = t_run;
            start_run = 0;
            if(stop_command_callback){
                stop_command_callback();
            }
        }

    }
    int isStartTrigger(){
        return start_run;
    }
    int isCommandTrigger(){
        return run_command;
    }
    void respond(int t_result){
        if(t_result && ! start_run){
            start_run = t_result;
            if(start_run_callback){
                start_run_callback();
            }
        }

        if(!t_result && start_run){
            start_run = t_result;
            if(stop_run_callback){
                stop_run_callback();
            }
        }
    }
};

int main(int argc, char **argv) {

    plog::RollingFileAppender<plog::CsvFormatter> fileAppender("scan_matching.csv", 800000, 10); // Create the 1st appender.
    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.


    std::vector<PointsStamped> laser_points_cache;

    //==== ros
    ros::init(argc, argv, "scan_matching");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    float sleep_time = 50.0;
    const char* sleep_time_param = "sleep_time";

    float tf_wait_time = 0.08;
    const char* tf_wait_time_param = "tf_wait_time";


    std::string file_dir = "data";
    const char* file_dir_param = "file_dir";

    std::string load_dir ;
    const char* load_dir_param = "load_dir";

    std::string mode = "map";
    const char* MODE_MAP = "map";
    const char* MODE_LOC = "loc";
    const char* mode_param = "mode";

    bool map_and_pub_loc = false;
    const char* map_and_pub_loc_param = "map_and_pub_loc";

    bool map_when_still = true;
    const char* map_when_still_param = "map_when_still";

    bool map_with_filter_scan = true;
    const char* map_with_filter_scan_param = "map_with_filter_scan";


    bool loc_and_map = false;
    const char* loc_and_map_param = "loc_and_map";


    const char* dump_data_param = "dump_data";
    bool dump_data = false;




    bool wait_extern_pose = true;
    const char* wait_extern_pose_param = "wait_extern_pose";

    float tf_stamp_tolerance_seconds = 0.05;
    const char* tf_stamp_tolerance_seconds_param = "tf_stamp_tolerance_seconds";


    float range_max = 30.0;
    const char* range_max_param = "range_max";
    float range_min = 1.0;
    const char* range_min_param = "range_min";

    float pcl_voxel_leaf_x = 0.05;
    float pcl_voxel_leaf_y = 0.05;
    float pcl_voxel_leaf_z = 0.05;
    const char* pcl_voxel_leaf_x_param = "pcl_voxel_leaf_x";
    const char* pcl_voxel_leaf_y_param = "pcl_voxel_leaf_y";
    const char* pcl_voxel_leaf_z_param = "pcl_voxel_leaf_z";
    int pcl_radius_neighbors = 3;
    float pcl_radius_radius = 0.1;

    float scan_point_jump = 0.06;
    float scan_noise_angle = 0.06;

    const char* scan_point_jump_param = "scan_point_jump";
    const char* scan_noise_angle_param = "scan_noise_angle";

    const char* pcl_radius_neighbors_param = "pcl_radius_neighbors";
    const char* pcl_radius_radius_param = "pcl_radius_radiusSearch";

    float pcl_norm_radius = 0.8;
    const char* pcl_norm_radius_param = "pcl_norm_radius";

    float pcl_cond_curvature = 0.2;

    const char* pcl_cond_curvature_param = "pcl_cond_curvature";


    float pcl_ref_voxel_leaf_x = 0.08;
    float pcl_ref_radius_radius = 0.1;
    int pcl_ref_radius_neighbors = 3;
    const char* pcl_ref_voxel_leaf_x_param = "pcl_ref_voxel_leaf_x";
    const char* pcl_ref_radius_radius_param = "pcl_ref_radius_radius";
    const char* pcl_ref_radius_neighbors_param = "pcl_ref_radius_neighbors";

    float pcl_icp_max_match_distance = 0.3;
    const char* pcl_icp_max_match_distance_param = "pcl_icp_max_match_distance";
    int pcl_icp_max_iter = 30;
    const char* pcl_icp_max_iter_param = "pcl_icp_max_iter";

    float pcl_icp_rotation_epsilon = 1e-6;
    float pcl_icp_translation_epsilon = 1e-6;
    const char* pcl_icp_rotation_epsilon_param = "pcl_icp_rotation_epsilon";
    const char* pcl_icp_translation_epsilon_param = "pcl_icp_translation_epsilon";

    float pcl_icp_ransac_outlier = 0.06;
    const char * pcl_icp_ransac_outlier_param = "pcl_icp_ransac_outlier";


    float no_move_translation = 5e-3;
    float no_move_rotation = 5e-3;

    float final_no_move_translation = 5e-3;
    float final_no_move_rotation = 5e-3;
    float no_move_ms = 500.0;

    int scan_filter_len = 20;
    float scan_filter_stddev = 0.005;
    const char* scan_filter_len_param = "scan_filter_len";
    const char* scan_filter_stddev_param = "scan_filter_stddev";



    const char* no_move_translation_param = "no_move_translation";
    const char* no_move_rotation_param = "no_move_rotation";
    const char* final_no_move_translation_param = "final_no_move_translation";
    const char* final_no_move_rotation_param = "final_no_move_rotation";
    const char* no_move_ms_param = "no_move_ms";


    float icp_reject_translation_jump = 0.1;
    float icp_reject_rotation_jump = 0.1;
    const char* icp_reject_translation_jump_param = "icp_reject_translation_jump";
    const char* icp_reject_rotation_jump_param = "icp_reject_rotation_jump";


    auto load_params = [&]{

        nh_private.getParam(map_and_pub_loc_param, map_and_pub_loc);
        nh_private.getParam(map_with_filter_scan_param, map_with_filter_scan);
        nh_private.getParam(map_when_still_param, map_when_still);

        nh_private.getParam(loc_and_map_param, loc_and_map);


        nh_private.getParam(sleep_time_param, sleep_time);

        nh_private.getParam(tf_wait_time_param, tf_wait_time);

        nh_private.getParam(scan_filter_len_param, scan_filter_len);
        nh_private.getParam(scan_filter_stddev_param, scan_filter_stddev);

        nh_private.getParam(tf_stamp_tolerance_seconds_param, tf_stamp_tolerance_seconds);

        nh_private.getParam(icp_reject_translation_jump_param, icp_reject_translation_jump);
        nh_private.getParam(icp_reject_rotation_jump_param, icp_reject_rotation_jump);

        nh_private.getParam(range_max_param, range_max);
        nh_private.getParam(range_min_param, range_min);
        nh_private.getParam(pcl_voxel_leaf_x_param, pcl_voxel_leaf_x);
        nh_private.getParam(pcl_voxel_leaf_y_param, pcl_voxel_leaf_y);
        nh_private.getParam(pcl_voxel_leaf_z_param, pcl_voxel_leaf_z);

        nh_private.getParam(pcl_radius_neighbors_param, pcl_radius_neighbors);
        nh_private.getParam(pcl_radius_radius_param, pcl_radius_radius);

        nh_private.getParam(scan_point_jump_param, scan_point_jump);
        nh_private.getParam(scan_noise_angle_param, scan_noise_angle);

        nh_private.getParam(pcl_norm_radius_param, pcl_norm_radius);


        nh_private.getParam(pcl_cond_curvature_param, pcl_cond_curvature);


        nh_private.getParam(pcl_ref_voxel_leaf_x_param, pcl_ref_voxel_leaf_x);
        nh_private.getParam(pcl_ref_radius_radius_param, pcl_ref_radius_radius);
        nh_private.getParam(pcl_ref_radius_neighbors_param, pcl_ref_radius_neighbors);

        nh_private.getParam(pcl_icp_max_match_distance_param, pcl_icp_max_match_distance);

        nh_private.getParam(pcl_icp_max_iter_param, pcl_icp_max_iter);
        nh_private.getParam(pcl_icp_rotation_epsilon_param, pcl_icp_rotation_epsilon);
        nh_private.getParam(pcl_icp_translation_epsilon_param, pcl_icp_translation_epsilon);
        nh_private.getParam(pcl_icp_ransac_outlier_param, pcl_icp_ransac_outlier);



        //
        nh_private.getParam(no_move_translation_param, no_move_translation);
        nh_private.getParam(no_move_rotation_param, no_move_rotation);
        nh_private.getParam(final_no_move_translation_param, final_no_move_translation);
        nh_private.getParam(final_no_move_rotation_param, final_no_move_rotation);
        nh_private.getParam(no_move_ms_param, no_move_ms);

        nh_private.getParam(file_dir_param, file_dir);
        nh_private.getParam(load_dir_param, load_dir);


        nh_private.getParam(mode_param, mode);

        nh_private.getParam(dump_data_param,dump_data);


        nh_private.getParam(wait_extern_pose_param,wait_extern_pose);

    };

    load_params();




    std::string cloud_filtered_topic = "cloud_filtered";
    std::string cloud_matched_topic = "cloud_matched";

    std::string cloud_reference_topic = "cloud_reference";
    std::string cloud_norm_topic = "cloud_norm";

    std::string icp_pose_topic = "robot_pose";
    std::string initialpose_topic = "/initialpose";

    std::string pose_trj_topic = "robot_pose_array";


    // ==== publisher
    // PointCloud2
    // tf: map_to_odom

//    bool tf_compute_map_odom = false;
    ros::Publisher cloud_filtered_pub = nh_private.advertise<sensor_msgs::PointCloud2>(cloud_filtered_topic, 1);
    ros::Publisher cloud_reference_pub = nh_private.advertise<sensor_msgs::PointCloud2>(cloud_reference_topic, 1);
    ros::Publisher cloud_matched_pub = nh_private.advertise<sensor_msgs::PointCloud2>(cloud_matched_topic, 1);

    ros::Publisher icp_pose_pub = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>(icp_pose_topic, 1);
    ros::Publisher inital_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic, 1);

    ros::Publisher robot_pose_trj_pub = nh_private.advertise<geometry_msgs::PoseArray>(pose_trj_topic, 1);

    ros::Publisher cloud_norm_pub = nh_private.advertise<geometry_msgs::PoseArray>(cloud_norm_topic, 1);


    geometry_msgs::PoseArray robot_pose_array;
    robot_pose_array.header.frame_id = "map";


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

    initialpose_msg.pose.covariance.fill(0.0);

    initialpose_msg.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
    initialpose_msg.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
    initialpose_msg.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;


    sensor_msgs::PointCloud2 cloud_filtered, cloud_reference, cloud_matched;
    geometry_msgs::PoseArray cloud_norm;


    cloud_norm.header.frame_id = "map";
    cloud_filtered.header.frame_id = "map";
    cloud_reference.header.frame_id = "map";
    cloud_matched.header.frame_id = "map";

    createPointCloud2(cloud_filtered, {"x", "y", "z"});
    createPointCloud2(cloud_reference, {"x", "y", "z"});
    createPointCloud2(cloud_matched, {"x", "y", "z"});


    //==== pcl cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud_norm(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud_norm_ref(new pcl::PointCloud<pcl::PointNormal>);


    //==== pcl
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutlierRemoval;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximateVoxelGrid;

    approximateVoxelGrid.setLeafSize(pcl_voxel_leaf_x, pcl_voxel_leaf_y, pcl_voxel_leaf_z);

    radiusOutlierRemoval.setRadiusSearch(pcl_radius_radius);
    radiusOutlierRemoval.setMinNeighborsInRadius(pcl_radius_neighbors);

//    radiusOutlierRemoval.setKeepOrganized(false);


    // icp
    pcl::PointCloud<pcl::PointXYZ> Final;
    pcl::PointCloud<pcl::PointNormal> Final_Norm;

    Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity ());
//    initial_guess(0,3) = 0.03;
//    initial_guess(1,3) = 0.02;


    pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;

    pcl::registration::WarpPointRigid3D<PointType, PointType>::Ptr warp_fcn
            (new pcl::registration::WarpPointRigid3D<PointType, PointType>);



    // Create a TransformationEstimationLM object, and set the warp to it
    pcl::registration::TransformationEstimation2D<PointType, PointType>::Ptr te_2d(
            new pcl::registration::TransformationEstimation2D<PointType, PointType>);



    // TransformationEstimationLM with warp_fcn is faster than TransformationEstimation2D
    pcl::registration::TransformationEstimationLM<PointType, PointType>::Ptr te(
            new pcl::registration::TransformationEstimationLM<PointType, PointType>);
    te->setWarpFunction(warp_fcn);


    // Pass the TransformationEstimation objec to the ICP algorithm
    icp.setTransformationEstimation(te);



//    icp.setMaximumIterations (10);
//    icp.setMaxCorrespondenceDistance (0.05);
//    icp.setRANSACOutlierRejectionThreshold (0.05);


// pl icp
    pcl::registration::WarpPointRigid3D<pcl::PointNormal, pcl::PointNormal>::Ptr warp_fcn_pl
            (new pcl::registration::WarpPointRigid3D<pcl::PointNormal, pcl::PointNormal>);
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> pl_icp;

    pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>::Ptr pl_te(
            new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>);

    pl_te->setWarpFunction(warp_fcn_pl);
    pl_icp.setTransformationEstimation(pl_te);



    // norm est
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    Normal2dEstimation norm_estim;
    norm_estim.setSearchMethod(tree);
    norm_estim.setRadiusSearch(pcl_norm_radius);
//    norm_estim.setKSearch(5);




// pcl tree
    float resolution = 0.05;



    pcl::octree::OctreePointCloudSearch<pcl::PointNormal> octree (resolution);



    // build the filter


    // ==== listener
    // LaserScan

    // tf: odom_to_base_link, base_link_to_base_laser


    bool scan_get_data = false;
    bool tf_get_base_laser = false;
//    bool tf_get_odom_base = false;

    sensor::ScanToPoints scan_handler;

    scan_handler.scan_max_jump = scan_point_jump;
    scan_handler.scan_noise_angle = scan_noise_angle;


    tf::TransformListener tl_;
    tf::TransformBroadcaster tf_br;
    tf::StampedTransform transform;

    tf::StampedTransform pub_map_odom_tf;
    tf::StampedTransform pub_reference_origin_tf;

    bool start_pub_tf = false;
    bool map_odom_tf_computed = false;

    ros::Duration transform_tolerance_;
    transform_tolerance_.fromSec(tf_stamp_tolerance_seconds);

    transform::Transform2d tf_base_laser;
    transform::Transform2d tf_odom_base;
    transform::Transform2d tf_map_base;


    const char* amcl_tf_broadcast = "/amcl/tf_broadcast";
    const char* status_param = "status";
    const char* run_param = "run";
    int run_command = 0;
    int run_command_last = 0;
    int start_run = 0;



    std::string base_frame = "base_link";
    std::string odom_frame = "odom";
    std::string laser_frame = "base_laser";
    std::string map_frame = "map";
    std::string reference_frame = "icp_origin";


    bool is_origin_compute = false;
    transform::Transform2d interactive_origin;
    ros_tool::InteractiveTf interactive_tf(map_frame, reference_frame);


    ros::Time scan_time;



    RangesFilter rangesFilter;
    rangesFilter.max_len = scan_filter_len;
    rangesFilter.max_stddev = scan_filter_stddev;

    MovementCheck movement_check;
    bool new_scan_filtered = false;


    movement_check.final_no_move_rotation_epsilon = final_no_move_rotation;
    movement_check.final_no_move_translation_epsilon = final_no_move_translation;
    movement_check.no_move_rotation_epsilon = no_move_rotation;
    movement_check.no_move_translation_epsilon = no_move_translation;
    movement_check.no_move_check_ms = no_move_ms;



    auto laser_cb = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        laser_frame.assign(msg->header.frame_id);
        scan_time = msg->header.stamp;
        scan_get_data = true;

#if 0
        scan_handler.getLocalPoints(msg->ranges, msg->angle_min, msg->angle_increment, range_min, range_max);

#endif
#if 1
        if(!movement_check.isStill()){

            scan_handler.getLocalPoints(msg->ranges, msg->angle_min, msg->angle_increment, range_min, range_max);

            rangesFilter.clear();
            new_scan_filtered = !map_with_filter_scan;
        }else{
            rangesFilter.add(msg->ranges);
            new_scan_filtered = rangesFilter.filter();
            if(new_scan_filtered){
                scan_handler.getLocalPoints(rangesFilter.getFiltered(), msg->angle_min, msg->angle_increment, range_min, range_max);
            }else{
                scan_handler.getLocalPoints(msg->ranges, msg->angle_min, msg->angle_increment, range_min, range_max);

            }


        }

        scan_get_data = scan_handler.range_valid_num > 100;

//        PLOGD << "movement_check.isStill(): " << movement_check.isStill() <<", range_valid_num:" << scan_handler.range_valid_num << ", update_index: " << rangesFilter.update_index << std::endl;

#endif


    };


    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laser_cb);
    nh.setParam(amcl_tf_broadcast, true);
    nh_private.setParam(status_param,0);



    transform::Transform2d tmp_icp(0.1, 0.05, 0.08);
    initial_guess(0, 0) = tmp_icp.matrix[0][0];
    initial_guess(0, 1) = tmp_icp.matrix[0][1];
    initial_guess(1, 0) = tmp_icp.matrix[1][0];
    initial_guess(1, 1) = tmp_icp.matrix[1][1];
    initial_guess(0, 3) = tmp_icp.matrix[0][2];
    initial_guess(1, 3) = tmp_icp.matrix[1][2];

//    tmp_icp = tmp_icp.inverse();


    PointCloudPoseSolver solver;

    auto &filter_reading_config = solver.getFilterReadingConfig();
    filter_reading_config.voxel_grid_size = pcl_voxel_leaf_x;
    filter_reading_config.radius_rmv_radius = pcl_radius_radius;
    filter_reading_config.radius_rmv_nn = pcl_radius_neighbors;

    filter_reading_config.cond_curve = pcl_cond_curvature;


    auto &norm_est_config = solver.getNormEstConfig();
    norm_est_config.radius = pcl_norm_radius;

    auto &filter_reference_config = solver.getFilterReferenceConfig();
    filter_reference_config.voxel_grid_size = pcl_ref_voxel_leaf_x;
    filter_reference_config.radius_rmv_radius = pcl_ref_radius_radius;
    filter_reference_config.radius_rmv_nn = pcl_ref_radius_neighbors;

    auto &icp_config = solver.getIcpConfig();

    icp_config.max_match_distance = pcl_icp_max_match_distance;
    icp_config.max_iter = pcl_icp_max_iter;
    icp_config.rotation_epsilon = pcl_icp_rotation_epsilon;
    icp_config.transformation_epsilon = pcl_icp_translation_epsilon;
    icp_config.ransac_outlier = pcl_icp_ransac_outlier;



    auto &move_check_config = solver.getMoveCheckConfig();
    move_check_config.final_no_move_rotation_epsilon = final_no_move_rotation;
    move_check_config.final_no_move_translation_epsilon = final_no_move_translation;
    move_check_config.no_move_rotation_epsilon = no_move_rotation;
    move_check_config.no_move_translation_epsilon = no_move_translation;
    move_check_config.no_move_check_ms = no_move_ms;

    auto& icp_rejection_config = solver.getErrorRejectionConfig();
    icp_rejection_config.max_rotation_jump = icp_reject_rotation_jump;
    icp_rejection_config.max_translation_jump = icp_reject_translation_jump;

#if 0
    solver.m_filter_reading_config.voxel_grid_size = pcl_voxel_leaf_x;
    solver.m_filter_reading_config.radius_rmv_radius = pcl_radius_radius;
    solver.m_filter_reading_config.radius_rmv_nn = pcl_radius_neighbors;

    solver.m_norm_est_config.radius = pcl_norm_radius;

    solver.m_filter_reference_config.voxel_grid_size = pcl_ref_voxel_leaf_x;
    solver.m_filter_reference_config.radius_rmv_radius = pcl_ref_radius_radius;
    solver.m_filter_reference_config.radius_rmv_nn = pcl_ref_radius_neighbors;

    solver.m_icp_config.max_match_distance = pcl_icp_max_match_distance;
#endif


    solver.config();

    bool is_map_update = true;

    auto load_solver_map = [&]{
        if(std::strcmp(mode.c_str(), MODE_LOC ) == 0){

            nh_private.getParam(file_dir_param, file_dir);
            nh_private.getParam(load_dir_param, load_dir);

            int rt = solver.loadFromFile(file_dir, load_dir);

            if(rt == -1){
                nh_private.setParam(status_param,-2);

                std::cerr << "solver load file error, exit" << std::endl;

                return -1;
            }else{
                is_map_update = true;
                start_pub_tf = false;
                return 0;
            }
        }else{
            return 0;

        }
    };

    start_run = load_solver_map();


    if((std::strcmp(mode.c_str(), MODE_MAP ) == 0) && !wait_extern_pose ){
        solver.setOrigin(transform::Transform2d());
    }

    nlohmann::json origin_json;
    transform::Transform2d load_origin_tf;

    bool is_interactive_tf_change = false;

    interactive_tf.setCallBack([&](auto &pose) {

        if (solver.isOriginCompute()) {
            is_interactive_tf_change = true;
            is_map_update = true;
            float yaw;
            to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, yaw);
            interactive_origin.set(pose.position.x, pose.position.y, yaw);
            solver.setOrigin(interactive_origin);

            robot_pose_array.poses.clear();

        }

    });

    tf::Transform temp_transform;
    tf::Quaternion temp_q;


    bool get_odom_base_tf = false;
    bool get_map_base_tf = false;


    common::TaskManager task_manager;
    common::TaskManager scan_task_manager;

    bool is_new_pose_compute = false;

    task_manager.addTask([&] {

        if(solver.isIcpFault() || !is_new_pose_compute){
            return true;
        }
//        PLOGD << "run task debug pointcloud"  << std::endl;

        auto stamp = ros::Time::now() +  transform_tolerance_;
        auto solver_cloud_reading = solver.getCloudReading();
        if(!solver_cloud_reading->empty()  ){
//            PLOGD << "cloud_norm_pub.publish(cloud_norm)"  << std::endl;

            PclPointCloudToPointCloud2(solver_cloud_reading, cloud_filtered);
            cloud_filtered.header.stamp = scan_time;
            cloud_filtered.header.frame_id.assign(laser_frame);
            cloud_filtered_pub.publish(cloud_filtered);

            PclPointCloudToPoseArray(solver_cloud_reading, cloud_norm);
            cloud_norm.header.stamp = scan_time;
            cloud_norm.header.frame_id.assign(laser_frame);
            cloud_norm_pub.publish(cloud_norm);

        }


        if(is_origin_compute){
            auto solver_cloud_matched = solver.getCloudMatched();
            if(!solver_cloud_matched->empty()){

//                PLOGD << "cloud_matched_pub.publish(cloud_matched)"  << std::endl;

                PclPointCloudToPointCloud2(solver_cloud_matched, cloud_matched);
                cloud_matched.header.stamp = stamp;
                cloud_matched.header.frame_id.assign(reference_frame);
                cloud_matched_pub.publish(cloud_matched);
            }


            if(is_map_update){
                auto solver_cloud_reference = solver.getCloudReference();
                if(!solver_cloud_reference->empty()){
//                    PLOGD << "cloud_reference_pub.publish(cloud_reference)"  << std::endl;

                    PclPointCloudToPointCloud2(solver_cloud_reference, cloud_reference);
                    cloud_reference.header.stamp = stamp ;
                    cloud_reference.header.frame_id.assign(reference_frame);
                    cloud_reference_pub.publish(cloud_reference);

                }
                is_map_update = false;
            }


        }



        auto &map_cloud_origin_tf = solver.getOrigin();



        if (solver.isOriginCompute() && !is_origin_compute   ) {
            is_origin_compute = true;

            if(!(std::strcmp(mode.c_str(), MODE_LOC) ==0  && !dump_data)){

                interactive_tf.setInitPose(map_cloud_origin_tf.x(), map_cloud_origin_tf.y(),
                                           map_cloud_origin_tf.yaw());
                interactive_tf.start(1.0, false);
            }
            robot_pose_array.poses.clear();
        }

        if(is_origin_compute){
            temp_transform.setOrigin(tf::Vector3(map_cloud_origin_tf.x(), map_cloud_origin_tf.y(), 0.0));
            temp_q.setRPY(0, 0, map_cloud_origin_tf.yaw());
            temp_transform.setRotation(temp_q);

            pub_reference_origin_tf = tf::StampedTransform(temp_transform,stamp,
                                                           map_frame, reference_frame);

            tf_br.sendTransform(pub_reference_origin_tf);



            if(is_new_pose_compute){
                is_new_pose_compute = false;
                if (robot_pose_array.poses.empty()) {
                    robot_pose_array.poses.push_back(initialpose_msg.pose.pose);
                } else {
                    float yaw, yaw_last;
                    to_yaw(robot_pose_array.poses.back().orientation.x, robot_pose_array.poses.back().orientation.y,
                           robot_pose_array.poses.back().orientation.z, robot_pose_array.poses.back().orientation.w, yaw_last);
                    to_yaw(initialpose_msg.pose.pose.orientation.x, initialpose_msg.pose.pose.orientation.y,
                           initialpose_msg.pose.pose.orientation.z, initialpose_msg.pose.pose.orientation.w, yaw);

                    if (std::abs(initialpose_msg.pose.pose.position.x - robot_pose_array.poses.back().position.x) > 0.03
                        ||
                        std::abs(initialpose_msg.pose.pose.position.y - robot_pose_array.poses.back().position.y) > 0.03
                        || std::abs(yaw_last - yaw) > 0.01
                            ) {
                        robot_pose_array.poses.push_back(initialpose_msg.pose.pose);
                    }
                }

                if (robot_pose_array.poses.size() > 200) {
                    robot_pose_array.poses.erase(robot_pose_array.poses.begin(), robot_pose_array.poses.begin() + 100);
                }
                robot_pose_array.header.stamp = stamp;
                if(!robot_pose_array.poses.empty()){
                    robot_pose_trj_pub.publish(robot_pose_array);

                }
            }

        }
        return true;
    }, 500.0);






    bool solver_need_reset = false;
#if 0
    task_manager.addTask([&]{
        return true;
    },50.0);
#endif

    common::Suspend suspend;
    MissionManager missionManager;

    missionManager.start_command_callback = [&]{
        start_run = 1;
        missionManager.respond(start_run);
        start_run = load_solver_map();
        missionManager.respond(start_run);
    };
    missionManager.stop_command_callback = [&]{
        nh.setParam(amcl_tf_broadcast, true);
        nh_private.setParam(status_param,0);
    };

    missionManager.start_run_callback = [&]{
        nh_private.setParam(status_param,1);
    };
    missionManager.stop_run_callback = [&]{
        nh_private.setParam(status_param,-1);
    };


    while (ros::ok()) {

        nh_private.getParam(run_param,run_command);

#if 1
        if(run_command == 0){
            run_command_last = run_command;
            nh.setParam(amcl_tf_broadcast, true);
            suspend.sleep(1000.0);
            continue;
        }
        if(run_command && !run_command_last){
            run_command_last = run_command;

            start_run = load_solver_map();
        }

        if(start_run == -1){
            suspend.sleep(1000.0);
            continue;
        }

#endif


#if 0
        missionManager.command(run_command);

        if(missionManager.isCommandTrigger() == 0 || missionManager.isStartTrigger() == 0){
            suspend.sleep(2000.0);
            continue;
        }

#endif

        ros::spinOnce();
        scan_task_manager.call();
        task_manager.call();

        if (!tf_get_base_laser && scan_get_data) {
            try {
                tl_.lookupTransform(base_frame, laser_frame, ros::Time(0), transform);
                tf_get_base_laser = true;
                tf_base_laser.set(transform.getOrigin().x(), transform.getOrigin().y(),
                                  tf::getYaw(transform.getRotation()));

                solver.setSensorRelativePose(tf_base_laser);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                suspend.sleep(sleep_time);
                continue;
            }
        }

        if(solver_need_reset){

//            PLOGD<<"wait amcl reset pose"<< std::endl;
            suspend.sleep(sleep_time);

            continue;
        }

        if (tf_get_base_laser && scan_get_data) {

            // 1.0 process sequence
            // 1.1 filter input(map&loc)
            // 1.2 match(map&loc)
            // 1.3 look up odom_base(map&loc)
            // 1.4 update solver odom_base and map(map)
            // 1.5 compute map_odom(loc)





            // LaserScan to PointCloud

//            LaserScanToPointCloud2(scan_handler.local_xy_points,scan_handler.range_valid_num,cloud_filtered);





            common::Time t1 = common::FromUnixNow();

            LaserScanToPclPointCloud(scan_handler.local_xy_points, scan_handler.range_valid_num, pcl_cloud_raw);


            solver.addSensorReading(pcl_cloud_raw);
            if(std::strcmp(mode.c_str(), MODE_MAP) ==0 ){
                solver.match_v2();

            }



            get_odom_base_tf = false;
            get_map_base_tf = false;
            // lookup odom base_link
            try {
                tl_.waitForTransform(odom_frame, base_frame, scan_time, ros::Duration(tf_wait_time));

                tl_.lookupTransform(odom_frame, base_frame, scan_time, transform);
//                tf_get_odom_base = true;
                tf_odom_base.set(transform.getOrigin().x(), transform.getOrigin().y(),
                                 tf::getYaw(transform.getRotation()));
                get_odom_base_tf = true;

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }

            try {
                tl_.waitForTransform(map_frame, base_frame, scan_time, ros::Duration(tf_wait_time));

                tl_.lookupTransform(map_frame, base_frame, scan_time, transform);
                tf_map_base.set(transform.getOrigin().x(), transform.getOrigin().y(),
                                tf::getYaw(transform.getRotation()));
                get_map_base_tf = true;

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }

            if(get_odom_base_tf){
                movement_check.checkStill(tf_odom_base);

                movement_check.checkMoveTrigger(tf_odom_base);

            }

            if (  (std::strcmp(mode.c_str(), MODE_MAP) ==0  )|| (  std::strcmp(mode.c_str(), MODE_LOC) ==0 && loc_and_map) ) {

//                PLOGD << "try add data to map: " << movement_check.isStill() << ", " << new_scan_filtered << std::endl;
//                PLOGD << "try add data to map: " << movement_check.isMoveTriggered() << ", " << new_scan_filtered << std::endl;

                if(
                        (movement_check.isMoveTriggered())
//                        && (movement_check.isStill() || !map_when_still)
                        && new_scan_filtered

                        )
                {
                    solver.matchUpdate();
//                    rangesFilter.clear();
                    is_map_update = true;
//                PLOGD << "try add cloud to map "<< std::endl;

                    movement_check.move_flag_last = movement_check.move_flag;
                }


            }

            if (std::strcmp(mode.c_str(), MODE_MAP) ==0  &&  get_map_base_tf) {
                solver.setRobotAbsolutePoseOrigin(tf_map_base);
                solver.setRobotAbsolutePoseInitIcp(tf_map_base);

//                start_pub_tf = (  std::strcmp(mode.c_str(), MODE_LOC) ==0   ) ;
                is_new_pose_compute = true;

            }
            if (std::strcmp(mode.c_str(), MODE_LOC) ==0  &&  get_map_base_tf) {
                solver.setRobotAbsolutePoseInitIcp(tf_map_base);
                is_new_pose_compute = true;

            }

            if(std::strcmp(mode.c_str(), MODE_LOC) ==0){
                solver.match_v2();

                start_pub_tf =  !solver.isIcpFault() && solver.isIcpWorking();

            }
            if( ( std::strcmp(mode.c_str(), MODE_MAP) ==0   &&  map_and_pub_loc && is_new_pose_compute)){
                start_pub_tf =  !solver.isIcpFault() && solver.isIcpWorking();

            }

            if(solver.isIcpWorking()){
                // get map odom tf and publish

//                std::cout << "map_odom_tf:\n" << map_odom_tf << std::endl;

                if (start_pub_tf  ) {
                    if(map_odom_tf_computed && !get_odom_base_tf){
                        pub_map_odom_tf.stamp_ = ros::Time::now() + transform_tolerance_;
                        tf_br.sendTransform(pub_map_odom_tf);
                        continue;
                    }
                    if(! map_odom_tf_computed && !get_odom_base_tf){
                        continue;
                    }

                    auto &map_odom_tf = solver.solveMapOdom(tf_odom_base);

                    temp_transform.setOrigin(tf::Vector3(map_odom_tf.x(), map_odom_tf.y(), 0.0));
                    temp_q.setRPY(0, 0, map_odom_tf.yaw());
                    temp_transform.setRotation(temp_q);

                    pub_map_odom_tf = tf::StampedTransform(temp_transform,
                                                           scan_time +
                                                           transform_tolerance_,
                                                           map_frame, odom_frame);


                    pub_map_odom_tf.stamp_ = ros::Time::now() + transform_tolerance_;
                    if(!map_odom_tf_computed){
                        PLOGD << "stop amcl "<<std::endl;
                        pub_map_odom_tf.stamp_ = ros::Time::now();

                    }
//                    PLOGD << "start scan_time :"<<scan_time<<std::endl;

//                    PLOGD << "start pub_map_odom_tf.stamp_:"<<pub_map_odom_tf.stamp_<<std::endl;
                    tf_br.sendTransform(pub_map_odom_tf);

                    nh.setParam(amcl_tf_broadcast, false);

                    map_odom_tf_computed = true;
                    nh_private.setParam(status_param,2);




                }else{
                    PLOGD << "stop pub_map_odom_tf "<<std::endl;

                    nh.setParam(amcl_tf_broadcast, true);
                    nh_private.setParam(status_param,-1);


                }


//                PLOGD << "solver.isIcpWorking() " << solver.isIcpWorking() << " solver.isIcpFault() " << solver.isIcpFault() << std::endl;

                if(solver.isIcpFault() ){
                    start_pub_tf = false;
                    solver_need_reset = true;
                    nh.setParam(amcl_tf_broadcast, true);
                    PLOGD << "addTask" << std::endl;

                    scan_task_manager.addTask([&]{
                        solver_need_reset = false;
                        PLOGD << "resetIcp" << std::endl;
                        solver.resetIcp();
                        return false;
                    }, 1000.0);
                }

                if(is_new_pose_compute && !solver.isIcpFault() && solver.isIcpWorking() && solver.isOriginCompute()){
                    auto &solved_pose = solver.getRobotAbsolutePose();
                    auto &origin_pose = solver.getOrigin();
                    auto &relative_pose  = solver.getRobotRelativePose();
//                    std::cout << "origin_pose \n" << origin_pose<< "\n";
//                    std::cout << "relative_pose \n" << relative_pose<< "\n";
//                    std::cout << "solved_pose \n" << solved_pose<< "\n";
//                    PLOGD << "icp solved_pose : " << solved_pose << std::endl;
//                    PLOGD << "icp origin_pose : " << origin_pose << std::endl;
//                    PLOGD << "icp relative_pose : " << relative_pose << std::endl;

                    initialpose_msg.pose.pose.position.x = solved_pose.x();
                    initialpose_msg.pose.pose.position.y = solved_pose.y();

                    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(solved_pose.yaw()),
                                          initialpose_msg.pose.pose.orientation);
                    initialpose_msg.header.stamp = scan_time;

//                    PLOGD<<"publish initialpose_msg: " << solved_pose << std::endl;

                    icp_pose_pub.publish(initialpose_msg);
                }




            }

//            std::cout << "preprocess time : " << common::ToMillSeconds(common::FromUnixNow() - t1) << " ms"  << std::endl;


            scan_get_data = false;
        } else {

            if (start_pub_tf && !solver.isIcpFault() ) {
                auto pub_stamp = ros::Time::now() + transform_tolerance_;

                if(pub_stamp > pub_map_odom_tf.stamp_){
                    pub_map_odom_tf.stamp_ =pub_stamp;
//                    PLOGD << "2 pub_map_odom_tf.stamp_:"<<pub_map_odom_tf.stamp_<<std::endl;
                    tf_br.sendTransform(pub_map_odom_tf);
                }
            }

            suspend.sleep(sleep_time);
        }


    }
    nh.setParam(amcl_tf_broadcast, true);
    nh_private.setParam(status_param,0);

    if(( (std::strcmp(mode.c_str(), MODE_MAP) ==0)  )|| ((std::strcmp(mode.c_str(), MODE_LOC) ==0) && dump_data && is_interactive_tf_change) ){
        std::string stamp = common::getCurrentDateTime("%Y-%m-%d-%H-%M-%S");

        int rt = solver.dumpToFile(file_dir,stamp);
        if(rt == 0){
            nh_private.setParam(load_dir_param, stamp);
        }else{
            nh_private.setParam(load_dir_param, "fail");

        }

    }
}