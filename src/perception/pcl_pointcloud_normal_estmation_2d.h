//
// Created by waxz on 23-2-4.
//

#ifndef SCAN_REPUBLISHER_PCL_POINTCLOUD_NORMAL_ESTMATION_2D_H
#define SCAN_REPUBLISHER_PCL_POINTCLOUD_NORMAL_ESTMATION_2D_H


#include "normal_estimation_2d.h"

#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/common/centroid.h>
#include <pcl/PointIndices.h>

#include <vector>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/search.h>
#include <pcl/PointIndices.h>
#include <pcl/common/pca.h>


namespace perception{


    template<typename PointT>
    inline void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::KdTreeFLANN<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
    }

    template<typename PointT>
    inline void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::search::KdTree<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
    }

    template<typename PointT>
    inline void createTree(typename pcl::PointCloud<PointT>::ConstPtr t_input_cloud, pcl::octree::OctreePointCloudSearch<PointT>& t_tree){
        t_tree.setInputCloud(t_input_cloud);
        t_tree.addPointsFromInputCloud ();
    }

    

    template<typename PointT, typename TreeType>
    class PointcloudNormal2dEstimation{

    public:
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;
        using Scalar = float;

    private:
        TreeType m_tree;
        float m_query_radius = 0.1;
        PointCloudConstPtr m_input_cloud;

        std::vector<int> m_query_indices;
        std::vector<float> m_query_distance;
        NormalEst2d m_NormalEst2d;

    public:
        PointcloudNormal2dEstimation(const TreeType& t_tree, float radius = 0.1, int num = 20):
                m_tree(t_tree),
                m_query_radius(radius),
                m_query_indices(num),
                m_query_distance(num){

        }

        void setQueryRadius(float radius ){
            m_query_radius = radius;
        }

        void setInputCloud(const PointCloudConstPtr& t_input_cloud)
        {
            m_input_cloud = t_input_cloud;
            createTree(m_input_cloud,m_tree );
        }

#if 0
        void compute_v1( const pcl::PointCloud<pcl::PointNormal>::Ptr&  output){

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
                    Eigen::Matrix <float, 2, 1> vp ( - query_point.x, - query_point.y);

                    // Dot product between the (viewpoint - point) and the plane normal
                    float cos_theta = vp.dot (normal);
                    // Flip the plane normal
                    if (cos_theta < 0)
                    {
                        nx *= -1;
                        ny *= -1;
                    }


//                    std::cout << __LINE__ << "eigenvalues:\n" << eigen_values << std::endl;
//                    std::cout << __LINE__ << "eigenvectors:\n" << eigen_vectors << std::endl;
                }


            }

        }

        void compute_v2( const pcl::PointCloud<pcl::PointNormal>::Ptr&  output){

            int input_point_num = m_input_cloud->points.size();
            output->points.resize(input_point_num,pcl::PointNormal(0.0,0.0,0.0,0.0,0.0,0.0));
            output->height = m_input_cloud->height;
            output->width = m_input_cloud->width;
            output->is_dense = true;

            int rt = 0;

            for(int i = 0 ; i < input_point_num;i++){


                auto& query_point = m_input_cloud->at(i);
                rt = m_tree.radiusSearch (query_point, m_query_radius, m_query_indices, m_query_distance);

                std::size_t point_count;
                point_count = m_query_indices.size ();
                m_NormalEst2d.reset();

                m_NormalEst2d.addCenter(m_input_cloud->at(m_query_indices[0]).x,m_input_cloud->at(m_query_indices[0]).y);

                for (const auto &index : m_query_indices)
                {
                    Scalar x = m_input_cloud->at(index).x , y = m_input_cloud->at(index).y ;
                    m_NormalEst2d.addPoints(x,y);
                }

                m_NormalEst2d.compute(output->points[i].normal_x,output->points[i].normal_y,output->points[i].normal_z,output->points[i].curvature);

            }

        }
#endif
        void compute( const pcl::PointCloud<pcl::PointNormal>::Ptr&  output){

            int input_point_num = m_input_cloud->points.size();
            output->points.resize(input_point_num,pcl::PointNormal(0.0,0.0,0.0,0.0,0.0,0.0));
            output->height = m_input_cloud->height;
            output->width = m_input_cloud->width;
            output->is_dense = true;

            int rt = 0;
            std::vector<int> query_indices;
            std::vector<float> query_distance;
            NormalEst2d normalEst2d;

#pragma omp parallel for \
firstprivate(normalEst2d,query_indices,query_distance)
            for(int i = 0 ; i < input_point_num;i++){
                rt = m_tree.radiusSearch (m_input_cloud->at(i), m_query_radius, query_indices, query_distance);
                if( query_indices.size () >3){
                    normalEst2d.addCenter(m_input_cloud->at(query_indices[0]).x,m_input_cloud->at(query_indices[0]).y);

                    for (const auto &index : query_indices)
                    {
                        Scalar x = m_input_cloud->at(index).x  , y = m_input_cloud->at(index).y ;
                        normalEst2d.addPoints(x,y);
                    }
                    normalEst2d.compute(output->points[i].normal_x,output->points[i].normal_y,output->points[i].normal_z,output->points[i].curvature);
                }else{
                    output->points[i].normal_x = output->points[i].normal_y = output->points[i].normal_z = output->points[i].curvature = std::numeric_limits<float>::quiet_NaN ();
                }
            }

        }

    };

}
#endif //SCAN_REPUBLISHER_PCL_POINTCLOUD_NORMAL_ESTMATION_2D_H
