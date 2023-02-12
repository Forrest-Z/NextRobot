
#include "pcl_circle_fitting.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>


namespace perception{
    void FindCirclePcl(std::vector<geometry::Point>& points,float radius, float edge_radius, float edge_range_offset, int min_point_num, float & center_x, float & center_y, float& error_mean){
        center_x = 100.0;
        center_y = 100.0;
        error_mean = 100.0;

        if(points.size() < min_point_num){

            return;
        }
//        std::sort(points.begin(),points.end(), [](auto& v1, auto& v2){ return v1.r < v2.r; });


        float angle_mean = std::atan2(points[0].y,points[0].x ),angle_sum = 0.0,range_sum = 0.0, range_mean = 0.0;
        for(auto&p : points){
            float normalise_angle = std::abs(p.b - angle_mean) < M_PI ? p.b : (p.b + ((p.b - angle_mean) > 0.0 ? -M_PI*2: M_PI*2) );
//        std::cout << "angle: " << p.b <<", normalise_angle: " << normalise_angle << "\n";

            angle_sum += normalise_angle;
            range_sum += p.r;

        }
        angle_mean = angle_sum / float(points.size());
        range_mean = range_sum/ float(points.size());
        float cx = range_mean*cos(angle_mean);
        float cy = range_mean*sin(angle_mean);



        pcl::PointCloud<pcl::PointXYZ>::Ptr init_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // populate our PointCloud with points
        init_cloud->width    = points.size();
        init_cloud->height   = 1;
        init_cloud->is_dense = true;
        init_cloud->points.resize (init_cloud->width * init_cloud->height);

        std::cout << "\nx_data = [";
        for(auto&p : points){
            std::cout << p.x << ", ";
        }
        std::cout <<"]\n";

        std::cout << "\ny_data = [";
        for(auto&p : points){
            std::cout << p.y << ", ";
        }
        std::cout <<"]\n";
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  start pcl fitting "  << std::endl;  ;


        for (pcl::index_t i = 0; i < init_cloud->size (); ++i){
            (*init_cloud)[i].x = points[i].x;
            (*init_cloud)[i].y = points[i].y;
        }

        std::vector<int> inliers;
        // created RandomSampleConsensus object and compute the appropriated model
        pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
                model_circle(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (init_cloud));

        model_circle->setRadiusLimits(radius-0.01,radius + 0.01);
        model_circle->setModelConstraints([cx,cy,range_mean](auto& x){

            if(x(0) < range_mean ||  ( (x(0) - cx) *(x(0) - cx)  + (x(1) - cy) *(x(1) - cy) ) > 0.05 ) {
                return false;
            }else{
                return true;
            }
            return true;

        });

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_circle);
        ransac.setDistanceThreshold (.04);
        ransac.computeModel();
        ransac.getInliers(inliers);
        Eigen::VectorXf model_coefficients;

        ransac.getModelCoefficients(model_coefficients);
        std::cout << "model_coefficients " << model_coefficients << std::endl;

        center_x = model_coefficients(0);
        center_y = model_coefficients(1);

        float error_sum = 0.0 ;


        error_sum = 0.0;
//    std::cout << "check error:\n";
        for(int i = 0 ; i <points.size();i++){
            auto& p = points[i];
            float d = std::sqrt( (p.x - center_x)*(p.x  -center_x) + (p.y - center_y)*(p.y - center_y)  );
//        std::cout << d << ", ";
            error_sum += std::abs(d- radius) ;
        }
        error_mean = error_sum/float(points.size());
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  init error_mean " << error_mean << std::endl;  ;

    }

}