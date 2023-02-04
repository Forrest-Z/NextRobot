//
// Created by Francois Gauthier-Clerc on 02/08/19.
//

#include "Normal2dEstimation.h"
#include<Eigen/Dense>

void Normal2dEstimation::setInputCloud(const ConstPtrCloud& cloud) {
    m_in_cloud = cloud;
    m_indices->clear();
    m_indices->resize (cloud->points.size ());
    for (unsigned int i = 0; i < cloud->points.size (); ++i) { (*m_indices)[i] = i; }
}


void Normal2dEstimation::setIndices(const pcl::PointIndices::Ptr& indices) {
    m_indices->clear();
    m_indices->resize(indices->indices.size());
    std::copy(indices->indices.cbegin(), indices->indices.cend(), m_indices->begin());
}

void Normal2dEstimation::setIndices(const pcl::PointIndices::ConstPtr& indices) {
    m_indices->clear();
    m_indices->resize(indices->indices.size());
    std::copy(indices->indices.cbegin(), indices->indices.cend(), m_indices->begin());
}


int Normal2dEstimation::searchForNeighbors(int index, std::vector<int>& nn_indices,std::vector<float>& nn_dists) const {
    nn_indices.clear();
    nn_dists.clear();
    if(m_k==0) {
        this->m_kd_tree->radiusSearch(index, m_search_radius, nn_indices, nn_dists, 0);
    }else {
        this->m_kd_tree->nearestKSearch(index, m_k, nn_indices, nn_dists);
    }
    return nn_indices.size();
}




void Normal2dEstimation::compute(const PtrCloud& normal_cloud) const {
// Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::shared_ptr<std::vector<int>> nn_indices(new std::vector<int>(m_k));
    std::vector<float> nn_dists(m_k);

    normal_cloud->points.resize(m_in_cloud->points.size());
    normal_cloud->height = m_in_cloud->height;
    normal_cloud->width = m_in_cloud->width;

    if ((m_k==0) && (m_search_radius==0)) {
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch !");
    }
    if ((m_k!=0) && (m_search_radius!=0)){
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch (not both) !");
    }

    this->m_kd_tree->setInputCloud(m_in_cloud, m_indices);

    normal_cloud->is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (m_in_cloud->is_dense) {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if(this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) ==0 ) {
//                std::cout << "1 norm " << __LINE__ << " nn_indices.size " << nn_indices->size() << std::endl;

                normal_cloud->points[idx].x = normal_cloud->points[idx].y = normal_cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();
                normal_cloud->is_dense = false;
                continue;
            }
//            std::cout << "2 norm " << __LINE__ << " nn_indices.size " << nn_indices->size() << std::endl;

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                       normal_cloud->points[idx].z);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                             normal_cloud->points[idx].z);

        }
    } else {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if (!isFinite(m_in_cloud->points[(*m_indices)[idx]]) ||
                this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) == 0) {
                normal_cloud->points[idx].x = normal_cloud->points[idx].y = normal_cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();

                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                       normal_cloud->points[idx].z);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                             normal_cloud->points[idx].z);
        }
    }
}



void Normal2dEstimation::compute(const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud) const {

    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::shared_ptr<std::vector<int>> nn_indices(new std::vector<int>(m_k));
    std::vector<float> nn_dists(m_k);

    normal_cloud->points.resize(m_in_cloud->points.size());
    normal_cloud->height = m_in_cloud->height;
    normal_cloud->width = m_in_cloud->width;

    if( (m_k==0) && (m_search_radius==0)){
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch !");
    }

    if( (m_k!=0) && (m_search_radius!=0)) {
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch (not both) !");
    }

    this->m_kd_tree->setInputCloud(m_in_cloud, m_indices);

    normal_cloud->is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (m_in_cloud->is_dense) {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if(this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) ==0 ) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();
                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                             normal_cloud->points[idx].normal_z,normal_cloud->points[idx].curvature);


        }
    } else {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if (!isFinite(m_in_cloud->points[(*m_indices)[idx]]) ||
                this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) == 0) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();

                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                             normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);
        }
    }

}

void Normal2dEstimation::compute(const pcl::PointCloud<pcl::PointNormal>::Ptr& normal_cloud) const {

    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::shared_ptr<std::vector<int>> nn_indices(new std::vector<int>(m_k));
    std::vector<float> nn_dists(m_k);

    normal_cloud->points.resize(m_in_cloud->points.size());
    normal_cloud->height = m_in_cloud->height;
    normal_cloud->width = m_in_cloud->width;

    if( (m_k==0) && (m_search_radius==0)){
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch !");
    }

    if( (m_k!=0) && (m_search_radius!=0)) {
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch (not both) !");
    }

    this->m_kd_tree->setInputCloud(m_in_cloud, m_indices);

    normal_cloud->is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (m_in_cloud->is_dense) {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if(this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) ==0 ) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();
                normal_cloud->is_dense = false;
                continue;
            }

//            std::cout << "1 check center: " << m_in_cloud->points[idx].x << ", " << m_in_cloud->points[idx].y << "\n";

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                             normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);

        }
    } else {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if (!isFinite(m_in_cloud->points[(*m_indices)[idx]]) ||
                this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) == 0) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();

                normal_cloud->is_dense = false;
                continue;
            }
//            std::cout << "2 check center: " << m_in_cloud->points[idx].x << ", " << m_in_cloud->points[idx].y << "\n";

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                             normal_cloud->points[idx].normal_z,normal_cloud->points[idx].curvature);
        }
    }

}

bool Normal2dEstimation::computePointNormal2d (std::shared_ptr<std::vector<int>> &indices,
                                               float &nx, float &ny, float &nz) const {

    if (indices->size () < 2)
    {
        nx = ny = nz = std::numeric_limits<float>::quiet_NaN ();
        return false;
    }
    if (indices->size()==2){
        double norm, vect_x, vect_y;
        vect_x = m_in_cloud->points[(*indices)[0]].x - m_in_cloud->points[(*indices)[1]].x;
        vect_y = m_in_cloud->points[(*indices)[0]].y - m_in_cloud->points[(*indices)[1]].y;
        norm = std::pow(std::pow(vect_x,2.0)+std::pow(vect_y,2.0), 0.5);
        vect_x /= norm;
        vect_y /= norm;
        nx = -vect_y;
        ny = vect_x;
        nz = 0.0;
        return true;
    }


    Eigen::MatrixX2f m(indices->size(),2);


    Eigen::Matrix2f covariance_matrix;

    Eigen::Matrix2f pointcloud_matrix;


#if 1

    int index = 0;
    for(int i : *indices){
        m(index,0) = m_in_cloud->points[i].x ;
        m(index,1) = m_in_cloud->points[i].y ;
        index++;
    }

    Eigen::VectorXf mean_vector = m.colwise().mean();

    Eigen::MatrixXf centered = m.rowwise() - mean_vector.transpose();

    Eigen::MatrixXf cov = (centered.adjoint() * centered) / ( m.rows() - 1 ) ;
//    std::cout << "mean_vector:\n"<< mean_vector << std::endl;
//    std::cout << "centered:\n"<< centered << std::endl;
//    std::cout << "cov:\n"<< cov << std::endl;
//    std::cout << "\ncomputePointNormal2d pca, pointcloud m = \n" << m << "\n";
//    std::cout << "m.colwise().mean()" << m.colwise().mean() << std::endl;
//    std::cout << "m.rowwise().mean()" << m.rowwise().mean() << std::endl;
//    std::cout << "cov" << cov << std::endl;
    // Eigenvector ayristirmasi
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig_solver(2);

    eig_solver.compute(cov);
//    std::cout << __LINE__ << "eigenvalues:\n" << eig_solver.eigenvalues() << std::endl;
//    std::cout << __LINE__ << "eigenvectors:\n" << eig_solver.eigenvectors() << std::endl;

    Eigen::Vector2f eigen_values = eig_solver.eigenvalues();
    Eigen::Matrix2f eigen_vectors = eig_solver.eigenvectors() ;

    if(std::abs(eigen_values(0)) < std::abs(eigen_values(1))){
        nx = eigen_vectors(0,0);
        ny = eigen_vectors(1,0);
    }else{
        nx = eigen_vectors(0,1);
        ny = eigen_vectors(1,1);
    }

    float n_angle = std::atan2(ny,nx);
    float center_angle = std::atan2(mean_vector(1),mean_vector(0));

    nz = 0.0;

    return true;

#endif


    // Get the plane normal and surface curvature
    PCA2D pca;

    pca.setInputCloud(m_in_cloud);
    pca.setIndices(indices);
    auto result = pca.getEigenVectors().col(1);
    nx = result(0);
    ny = result(1);
    nz = 0.0;
    return true;

}

bool Normal2dEstimation::computePointNormal2d (std::shared_ptr<std::vector<int>>& indices,
                           float &nx, float &ny, float &nz, float& curvature) const {
    if (indices->size () < 2)
    {
        nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
        return false;
    }
    if (indices->size()==2){
        double norm, vect_x, vect_y;
        vect_x = m_in_cloud->points[(*indices)[0]].x - m_in_cloud->points[(*indices)[1]].x;
        vect_y = m_in_cloud->points[(*indices)[0]].y - m_in_cloud->points[(*indices)[1]].y;
        norm = std::pow(std::pow(vect_x,2.0)+std::pow(vect_y,2.0), 0.5);
        vect_x /= norm;
        vect_y /= norm;
        nx = -vect_y;
        ny = vect_x;
        nz = 0.0;
        curvature = 0.0;
        return true;
    }





    Eigen::MatrixX2f m(indices->size(),2);


    Eigen::Matrix2f covariance_matrix;

    Eigen::Matrix2f pointcloud_matrix;


#if 1

    int index = 0;
    for(int i : *indices){
        m(index,0) = m_in_cloud->points[i].x ;
        m(index,1) = m_in_cloud->points[i].y ;
        index++;
    }

    Eigen::VectorXf mean_vector = m.colwise().mean();

    Eigen::MatrixXf centered = m.rowwise() - mean_vector.transpose();

    Eigen::MatrixXf cov = (centered.adjoint() * centered) / ( m.rows() - 1 ) ;
//    std::cout << "mean_vector:\n"<< mean_vector << std::endl;
//    std::cout << "centered:\n"<< centered << std::endl;
//    std::cout << "cov:\n"<< cov << std::endl;
//    std::cout << "\ncomputePointNormal2d pca, pointcloud m = \n" << m << "\n";
//    std::cout << "m.colwise().mean()" << m.colwise().mean() << std::endl;
//    std::cout << "m.rowwise().mean()" << m.rowwise().mean() << std::endl;
//    std::cout << "cov" << cov << std::endl;
    // Eigenvector ayristirmasi
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig_solver(2);

    eig_solver.compute(cov);
//    std::cout << __LINE__ << "eigenvalues:\n" << eig_solver.eigenvalues() << std::endl;
//    std::cout << __LINE__ << "eigenvectors:\n" << eig_solver.eigenvectors() << std::endl;

    Eigen::Vector2f eigen_values = eig_solver.eigenvalues();
    Eigen::Matrix2f eigen_vectors = eig_solver.eigenvectors() ;

    if(std::abs(eigen_values(0)) < std::abs(eigen_values(1))){
        nx = eigen_vectors(0,0);
        ny = eigen_vectors(1,0);
    }else{
        nx = eigen_vectors(0,1);
        ny = eigen_vectors(1,1);
    }

    float n_angle = std::atan2(ny,nx);
    float center_angle = std::atan2(mean_vector(1),mean_vector(0));

    nz = 0.0;

    return true;

#endif
    // Get the plane normal and surface curvature
    PCA2D pca;
//    std::cout << "\ncomputePointNormal2d pca, list norm points:\n" << m << "\n";
//
//    // Matrisin aritmetik ortalamasi
//    std::cout << "m.mean() " << m.mean() << std::endl;
//    // Matris sutunlarinin aritmetik ortalamasi



//    std::cout << "list end\n";

    pca.setInputCloud(m_in_cloud);
    pca.setIndices(indices);
    auto result = pca.getEigenVectors().col(1);
    nx = result(0);
    ny = result(1);
    nz = 0.0;
    std::cout << "get norm: " << nx << ", " << ny << "\n";

    curvature = pca.getEigenValues()(1) /  (pca.getEigenValues()(0)  + pca.getEigenValues()(1) );
    return true;

}

bool Normal2dEstimation::computePointNormal2d (std::shared_ptr<std::vector<int>> &indices,
                                               Eigen::Vector3f &line_parameters) const {

    if (indices->size () < 2)
    {
        line_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
        return false;
    }


    if (indices->size()==2){
        double norm, vect_x, vect_y;
        vect_x = m_in_cloud->points[(*indices)[0]].x - m_in_cloud->points[(*indices)[1]].x;
        vect_y = m_in_cloud->points[(*indices)[0]].y - m_in_cloud->points[(*indices)[1]].y;
        norm = std::pow(std::pow(vect_x,2.0)+std::pow(vect_y,2.0), 0.5);
        vect_x /= norm;
        vect_y /= norm;
        line_parameters(0) = -vect_y;
        line_parameters(1) = vect_x;
        line_parameters(2) = 0.0;
        return true;
    }

#if 1
    Eigen::MatrixX2f m(indices->size(),2);


    Eigen::Matrix2f covariance_matrix;

    Eigen::Matrix2f pointcloud_matrix;
    int index = 0;
    for(int i : *indices){
        m(index,0) = m_in_cloud->points[i].x ;
        m(index,1) = m_in_cloud->points[i].y ;
        index++;
    }

    Eigen::VectorXf mean_vector = m.colwise().mean();

    Eigen::MatrixXf centered = m.rowwise() - mean_vector.transpose();

    Eigen::MatrixXf cov = (centered.adjoint() * centered) / ( m.rows() - 1 ) ;
//    std::cout << "mean_vector:\n"<< mean_vector << std::endl;
//    std::cout << "centered:\n"<< centered << std::endl;
//    std::cout << "cov:\n"<< cov << std::endl;
//    std::cout << "\ncomputePointNormal2d pca, pointcloud m = \n" << m << "\n";
//    std::cout << "m.colwise().mean()" << m.colwise().mean() << std::endl;
//    std::cout << "m.rowwise().mean()" << m.rowwise().mean() << std::endl;
//    std::cout << "cov" << cov << std::endl;
    // Eigenvector ayristirmasi
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig_solver(2);

    eig_solver.compute(cov);
//    std::cout << __LINE__ << "eigenvalues:\n" << eig_solver.eigenvalues() << std::endl;
//    std::cout << __LINE__ << "eigenvectors:\n" << eig_solver.eigenvectors() << std::endl;

    Eigen::Vector2f eigen_values = eig_solver.eigenvalues();
    Eigen::Matrix2f eigen_vectors = eig_solver.eigenvectors() ;

    if(std::abs(eigen_values(0)) < std::abs(eigen_values(1))){
        line_parameters(0) = eigen_vectors(0,0);
        line_parameters(1) = eigen_vectors(0,1);
    }else{
        line_parameters(0) = eigen_vectors(1,0);
        line_parameters(1) = eigen_vectors(1,1);
    }

    float n_angle = std::atan2(line_parameters(1),line_parameters(0));
    float center_angle = std::atan2(mean_vector(1),mean_vector(0));

    line_parameters(2) = 0.0;

    return true;

#endif
    pcl::PCA<Point> pca;

    pca.setInputCloud(m_in_cloud);
    pca.setIndices(indices);

    auto result = pca.getEigenVectors().col(1);

    line_parameters(0) = result(0);
    line_parameters(1) = result(1);
    line_parameters(2) = 0.0;
    return true;

}



void Normal2dEstimation::flipNormalTowardsViewpoint (const Point &point, float& x, float& y, float& z ) const {
    Eigen::Matrix <double, 3, 1> normal (x, y, z);
    Eigen::Matrix <double, 3, 1> vp (m_view_point.x - point.x, m_view_point.y - point.y, 0.);

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = vp.dot (normal);
    // Flip the plane normal
    if (cos_theta < 0)
    {
        x *= -1;
        y *= -1;
        z *= -1;
    }
}

void Normal2dEstimation::flipNormalTowardsViewpoint (const Point &point, float& x, float& y, float& z,float &c) const {
    Eigen::Matrix <double, 3, 1> normal (x, y, z);


    Eigen::Matrix <double, 3, 1> vp (m_view_point.x - point.x, m_view_point.y - point.y, 0.);
    normal.normalize();
    vp.normalize();
    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = vp.dot (normal);
    // Flip the plane normal
    if (cos_theta < 0)
    {
        x *= -1;
        y *= -1;
        z *= -1;
    }
    c = std:: acos(std::abs(cos_theta));
}





void Normal2dEstimation::flipNormalTowardsViewpoint (const Point &point, Eigen::Matrix<double, 3, 1>& normal) const {
    Eigen::Matrix <double, 3, 1> vp (m_view_point.x - point.x, m_view_point.y - point.y, 0.);

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = vp.dot (normal);

    // Flip the plane normal
    if (cos_theta < 0)
    {
        normal *= -1;

    }
}