//
// Created by waxz on 23-2-4.
//

#ifndef SCAN_REPUBLISHER_POINTCLOUD_PCL_ROS_H
#define SCAN_REPUBLISHER_POINTCLOUD_PCL_ROS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"

namespace sensor{


    // PointCloud to
    inline void PclPointCloudToPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, sensor_msgs::PointCloud2 &cloud) {
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
    inline void PclPointCloudToPointCloud2(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr, sensor_msgs::PointCloud2 &cloud) {
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

    inline void PclPointCloudToPoseArray(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr, geometry_msgs::PoseArray &pose_array) {


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
            math::yaw_to_quaternion(yaw, pose_array.poses[i].orientation.x, pose_array.poses[i].orientation.y,
                                    pose_array.poses[i].orientation.z, pose_array.poses[i].orientation.w);

        }
//    std::cout << "\nend show all yaw\n";

    }


}
#endif //SCAN_REPUBLISHER_POINTCLOUD_PCL_ROS_H
