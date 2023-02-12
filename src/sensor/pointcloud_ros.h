//
// Created by waxz on 23-1-12.
//

#ifndef SCAN_REPUBLISHER_POINTCLOUD_ROS_H
#define SCAN_REPUBLISHER_POINTCLOUD_ROS_H

#include "geometry_types.h"
#include "sensor_msgs/PointCloud2.h"

namespace sensor{


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

    void LaserScanToPointCloud2(const std::vector<float> &scan_points, int point_num, sensor_msgs::PointCloud2 &cloud) {
        int point_num_ = 0.5 * (scan_points.size());
        if (point_num_ < point_num) {
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


    void LaserScanToPointCloud2(const std::vector<geometry::Point> &scan_points, int point_num, sensor_msgs::PointCloud2 &cloud) {
        int point_num_ = (scan_points.size());
        if (point_num_ < point_num) {
            return;
        }
        point_num = point_num<= point_num_ ? point_num:point_num_;

        cloud.row_step = point_num * cloud.point_step;
        cloud.data.resize(cloud.row_step);
        cloud.width = point_num;
        for (int i = 0; i < point_num; i++) {
            uint8_t *data_pointer = &cloud.data[0] + i * cloud.point_step;
            *(float *) data_pointer = scan_points[i].x;
            data_pointer += 4;
            *(float *) data_pointer = scan_points[i].y;
            data_pointer += 4;
            *(float *) data_pointer = 0.0;

        }
    }

    void LaserScanToPointCloud2Append(const std::vector<geometry::Point> &scan_points, int point_num, sensor_msgs::PointCloud2 &cloud) {
        int point_num_ = (scan_points.size());
        point_num = point_num<= point_num_ ? point_num:point_num_;

        cloud.row_step += point_num * cloud.point_step;
        cloud.data.resize(cloud.row_step);
        int cloud_old_width = cloud.width;
        cloud.width += point_num;
        for (int i = cloud_old_width; i < cloud.width; i++) {
            uint8_t *data_pointer = &cloud.data[0] + i * cloud.point_step;
            *(float *) data_pointer = scan_points[i].x;
            data_pointer += 4;
            *(float *) data_pointer = scan_points[i].y;
            data_pointer += 4;
            *(float *) data_pointer = 0.0;

        }
    }


}

#endif //SCAN_REPUBLISHER_POINTCLOUD_ROS_H
