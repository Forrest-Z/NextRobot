//
// Created by waxz on 23-2-4.
//

#ifndef SCAN_REPUBLISHER_POINTCLOUD_PCL_H
#define SCAN_REPUBLISHER_POINTCLOUD_PCL_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
namespace sensor{
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
}
#endif //SCAN_REPUBLISHER_POINTCLOUD_PCL_H
