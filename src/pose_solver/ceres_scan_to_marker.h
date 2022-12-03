//
// Created by waxz on 22-10-12.
//

#ifndef SCAN_REPUBLISHER_CERES_SCAN_TO_MARKER_H
#define SCAN_REPUBLISHER_CERES_SCAN_TO_MARKER_H

#include <deque>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

#include "common/type_cast.h"
#include <Eigen/Dense>





namespace pose_solver{
    using ceres::CENTRAL;
    using ceres::CostFunction;
    using ceres::NumericDiffCostFunction;
    using ceres::Problem;
    using ceres::Solve;
    using ceres::Solver;

    inline ceres::Solver::Options createOption() {
        ceres::Solver::Options ceres_solver_options;
        ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        ceres_solver_options.minimizer_progress_to_stdout = true;
        ceres_solver_options.num_threads = 1;
        ceres_solver_options.function_tolerance = 1e-4;  // Enough for denoising.
        ceres_solver_options.max_num_iterations = 50;
//        ceres_solver_options.parameter_tolerance = 1e-3;

//        ceres_solver_options.minimizer_type = ceres::LINE_SEARCH;
//        ceres_solver_options.logging_type = ceres::LoggingType::SILENT;

        return ceres_solver_options;

    }


    // given pointcloud_cluster and markers
    // solve nest laser_pose= [x,y,yaw] to transform markers to fit pointcloud_cluster

    struct CostFunctor {

        std::vector<double> markers;
//        std::vector<double> cache_markers_double;
//        std::vector<ceres::Jet<double,3>> cache_markers_jet;
        int marker_num;
        constexpr  static size_t MAX_ARRAY_SIZE = 500;

        std::vector<std::vector<std::array<double,3>>> clusters;

        template<typename FloatType>
        CostFunctor(const std::vector<FloatType> &  markers_,
                    const std::vector<std::vector<std::array<FloatType,3>>>&  clusters_
                    ) {

            common::simple_cast(markers,markers_);
            common::simple_cast(clusters,clusters_);
            marker_num= markers.size()/2;
            if(markers.size() >MAX_ARRAY_SIZE){
                char msg[200];
                sprintf(msg, "Error, %s , %d, array size too large, %ld > %ld ",__FILE__, __LINE__ ,markers.size(), MAX_ARRAY_SIZE);
                throw std::logic_error(msg);

            }


        }
        template<typename FloatType>
        static CostFunction* create(const std::vector<FloatType> &  markers_,
                                    const std::vector<std::vector<std::array<FloatType,3>>>&  clusters_){


            return new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 1, 3>(new CostFunctor(markers_, clusters_));
        }

        template <typename T>
        bool operator()(const T* const parameters, T* residual) const {


            const T& tx = parameters[0];
            const T& ty = parameters[1];
            const T& yaw = parameters[2];

            T r00 =  cos(yaw);
            T r01 =  -sin(yaw);
            T r10 = sin(yaw);
            T r11   = cos(yaw);

//            std::cout << "ceres, tx = " << tx << ", ty = " << ty << ", yaw = " << yaw << std::endl;
//            std::cout << "ceres, r00 = " << r00 << ", r01 = " << r01 << ", r10 = " << r10  << ", r11 = " << r11 << std::endl;

            const double* p_data_x = &(markers[0]);

            T temp_data_array[MAX_ARRAY_SIZE];

            for (int i = 0; i < marker_num; i++) {
                temp_data_array[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
                temp_data_array[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== ceres p_x = " << p_data_x[i+i] << ", p_y = " << p_data_x[i + i + 1] << std::endl;
//                std::cout << "== ceres d_x = " << r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx << ", d_y = " << r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty << std::endl;

            }

            residual[0] = 0.0;

            for (int i = 0; i < marker_num; i++) {
                if(clusters[i].empty())continue;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== ceres m_x = " << temp_data_array[i + i]<< ", m_y = " << temp_data_array[i + i + 1] << std::endl;
                T temp_residual = 0.0;
                for(auto& e: clusters[i]){
                    temp_residual += std::abs(e[0] - temp_data_array[i + i]) + std::abs(e[1] - temp_data_array[i + i + 1]);
//                    std::cout << "== ceres c_x = " << e[0] << ", c_y= " << e[1] << std::endl;
                }
                residual[0] += temp_residual/clusters[i].size();
            }
            return true;
        }
    };




    /*
     each block contains a single marker
     */
    struct MarkerFitCalib {

        std::vector<std::array<double,3>> clusters;
        double marker_radius = 0.05;

        double trunc_radius = 0.05;
        double trunc_weight = 0.5;



        MarkerFitCalib(const std::vector<std::array<float,3>>& cluster_, float marker_radius_){

            common::simple_cast(clusters,cluster_ );
            marker_radius = marker_radius_;


        }
        template<typename ...ARGS>
        static CostFunction* create(ARGS ...args){
            return  new NumericDiffCostFunction<MarkerFitCalib, CENTRAL, 1, 3,3>(new MarkerFitCalib(args...));
        }



        template<typename T>
        bool operator()(const T *const laser_pose_inv, const T *const marker_pose_in_map, T *residual) const {


            const T& tx = laser_pose_inv[0];
            const T& ty = laser_pose_inv[1];
            const T& yaw = laser_pose_inv[2];

            T r00 =  cos(yaw);
            T r01 =  -sin(yaw);
            T r10 = sin(yaw);
            T r11   = cos(yaw);
            // use laser_pose_inv to transform marker_pose_in_map to marker_pose_in_laser

            // 2 cast for marker fitting
            T temp_data_array[2];
            T temp_edge_data_array[2];

            temp_data_array[0] = r00 * marker_pose_in_map[0] + r01 * marker_pose_in_map[1] + tx;
            temp_data_array[1] = r10 * marker_pose_in_map[0] + r11 * marker_pose_in_map[1] + ty;
            T range = ceres::sqrt(temp_data_array[0]*temp_data_array[0] + temp_data_array[1]*temp_data_array[1]) ;
            temp_edge_data_array[0] =  temp_data_array[0]*(range - marker_radius)/range;

            temp_edge_data_array[1] =  temp_data_array[1]*(range - marker_radius)/range;





            T cost_edge = 0.0;
            T cost_radius = 0.0;
            T cost_edge_min = 100.0;
            T dist = 0.0;

            T temp_residual = 0.0;
            residual[0] = 0.0;

            for(auto& e: clusters){
                dist =   ceres::sqrt( (e[0] - temp_edge_data_array[0])*(e[0] - temp_edge_data_array[0]) +  (e[1] - temp_edge_data_array[1])* (e[1] - temp_edge_data_array[1]))   ;

                cost_edge_min = (dist < cost_edge_min) ? dist : cost_edge_min;

                cost_edge += (dist>trunc_radius) ? (trunc_radius + trunc_weight*(dist-trunc_radius)) : (dist);
//                    std::cout << "== ceres c_x = " << e[0] << ", c_y= " << e[1] << std::endl;
            }
            if(cost_edge_min < 0.02){
                for(auto& e: clusters){
                    dist =  ceres::abs(ceres::sqrt( (e[0] - temp_data_array[0])*(e[0] - temp_data_array[0]) +  (e[1] - temp_data_array[1])* (e[1] - temp_data_array[1])) - marker_radius)  ;


                    cost_radius += (dist>trunc_radius) ? (trunc_radius + trunc_weight*(dist-trunc_radius)) : (dist);
//                    std::cout << "== ceres c_x = " << e[0] << ", c_y= " << e[1] << std::endl;
                }
                cost_edge *= 0.2;
            }



            residual[0] = (cost_edge + cost_radius)/clusters.size();

            return true;
        }

    };
    struct MarkerFit {

        std::vector<std::array<double,3>> clusters;

        std::array<double,3> marker_pose_in_map;
        double marker_radius = 0.05;
        double trunc_radius = 0.05;
        double trunc_weight = 0.5;
        double weight = 1.0;


        MarkerFit(const std::vector<std::array<float,3>>& cluster_, float marker_radius_,std::array<double,3> marker_pose_in_map_,double weight_){

            common::simple_cast(clusters,cluster_ );
            marker_radius = marker_radius_;
            common::simple_cast(marker_pose_in_map,marker_pose_in_map_ );

            weight = weight_;
        }
        template<typename ...ARGS>
        static CostFunction* create(ARGS ...args){
            return  new NumericDiffCostFunction<MarkerFit, CENTRAL, 1, 3>(new MarkerFit(args...));
        }

        template<typename T>
        bool operator()(const T *const laser_pose_inv, T *residual) const {


            const T& tx = laser_pose_inv[0];
            const T& ty = laser_pose_inv[1];
            const T& yaw = laser_pose_inv[2];

            T r00 =  cos(yaw);
            T r01 =  -sin(yaw);
            T r10 = sin(yaw);
            T r11   = cos(yaw);
            // use laser_pose_inv to transform marker_pose_in_map to marker_pose_in_laser

            // 2 cast for marker fitting
            T temp_data_array[2];
            T temp_edge_data_array[2];

            temp_data_array[0] = r00 * marker_pose_in_map[0] + r01 * marker_pose_in_map[1] + tx;
            temp_data_array[1] = r10 * marker_pose_in_map[0] + r11 * marker_pose_in_map[1] + ty;
            T range = ceres::sqrt(temp_data_array[0]*temp_data_array[0] + temp_data_array[1]*temp_data_array[1]) ;
            temp_edge_data_array[0] =  temp_data_array[0]*(range - marker_radius)/range;

            temp_edge_data_array[1] =  temp_data_array[1]*(range - marker_radius)/range;





            T cost_edge = 0.0;
            T cost_radius = 0.0;
            T cost_edge_min = 100.0;
            T dist = 0.0;

            T temp_residual = 0.0;
            residual[0] = 0.0;
#if 0
            for(auto& e: clusters){
                dist =   ceres::sqrt( (e[0] - temp_edge_data_array[0])*(e[0] - temp_edge_data_array[0]) +  (e[1] - temp_edge_data_array[1])* (e[1] - temp_edge_data_array[1]))   ;

                cost_edge_min = (dist < cost_edge_min) ? dist : cost_edge_min;

                cost_edge += (dist>trunc_radius) ? (trunc_radius + trunc_weight*(dist-trunc_radius)) : (dist);
//                    std::cout << "== ceres c_x = " << e[0] << ", c_y= " << e[1] << std::endl;
            }

            if(cost_edge_min < 0.02){
                for(auto& e: clusters){
                    dist =  ceres::abs(ceres::sqrt( (e[0] - temp_data_array[0])*(e[0] - temp_data_array[0]) +  (e[1] - temp_data_array[1])* (e[1] - temp_data_array[1])) - marker_radius)  ;


                    cost_radius += (dist>trunc_radius) ? (trunc_radius + trunc_weight*(dist-trunc_radius)) : (dist);
//                    std::cout << "== ceres c_x = " << e[0] << ", c_y= " << e[1] << std::endl;
                }
                cost_edge *= 0.5;

            }
#endif



            for(auto& e: clusters){
                dist =   ceres::sqrt( (e[0] - temp_data_array[0])*(e[0] - temp_data_array[0]) +  (e[1] - temp_data_array[1])* (e[1] - temp_data_array[1]))     ;


                cost_radius += (dist>trunc_radius) ? (trunc_radius + trunc_weight*(dist-trunc_radius)) : (dist);
//                    std::cout << "== ceres c_x = " << e[0] << ", c_y= " << e[1] << std::endl;
            }

            residual[0] = weight*(cost_edge + cost_radius)/clusters.size();
            return true;
        }

    };


    /*

     pose constrains

     odom_base_tf_1 , odom_base_tf_2

     laser_pose_1,    laser_pose_2

     transform laser_pose to base_pose = laser_pose * base_laser_tf_inv


     */
    // Normalizes the angle in radians between [-pi and pi).
    template <typename T>
    inline T NormalizeAngle(const T& angle_radians) {
        // Use ceres::floor because it is specialized for double and Jet types.
        T two_pi(2.0 * M_PI);
        return angle_radians -
               two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
    }



    struct PoseConstrains{

        Eigen::Matrix<double,3,3> a_Tcap_b;
        Eigen::Matrix<double,3,3> base_Tcap_laser;


        double weight_x = 1.0;
        double weight_y = 1.0;
        double weight_yaw = 1.0;


        template<typename ...ARGS>
        static CostFunction* create(ARGS ...args){
            return  new NumericDiffCostFunction<PoseConstrains, CENTRAL, 3, 3,3>(new PoseConstrains(args...));
        }


        PoseConstrains(double * constrains, double * base_to_laser, double weight_x_ = 1.0, double weight_y_ = 1.0,double weight_yaw_=1.0){
            using T=double;
            weight_x = weight_x_;
            weight_y = weight_y_;
            weight_yaw = weight_yaw_;

            {
                T cos_t = T(cos( constrains[2]));
                T sin_t = T(sin( constrains[2] ));
                a_Tcap_b(0,0) = cos_t;
                a_Tcap_b(0,1) = -sin_t;
                a_Tcap_b(1,0) = sin_t;
                a_Tcap_b(1,1) = cos_t;
                a_Tcap_b(0,2) = constrains[0];
                a_Tcap_b(1,2) = constrains[1];

                a_Tcap_b(2,0) = T(0.0);
                a_Tcap_b(2,1) = T(0.0);
                a_Tcap_b(2,2) = T(1.0);
            }

            {
                T cos_t = T(cos( base_to_laser[2]));
                T sin_t = T(sin( base_to_laser[2] ));
                base_Tcap_laser(0,0) = cos_t;
                base_Tcap_laser(0,1) = -sin_t;
                base_Tcap_laser(1,0) = sin_t;
                base_Tcap_laser(1,1) = cos_t;
                base_Tcap_laser(0,2) = base_to_laser[0];
                base_Tcap_laser(1,2) = base_to_laser[1];

                base_Tcap_laser(2,0) = T(0.0);
                base_Tcap_laser(2,1) = T(0.0);
                base_Tcap_laser(2,2) = T(1.0);
            }


        }
        template<typename T>
        bool operator()(const T *const P1,const T *const P2, T *residual)const{


            // Convert P1 to T1 ^w_T_a
            Eigen::Matrix<T,3,3> w_T_a;
            {
                T cos_t = T(cos( P1[2] ));
                T sin_t = T(sin( P1[2] ));
                w_T_a(0,0) = cos_t;
                w_T_a(0,1) = -sin_t;
                w_T_a(1,0) = sin_t;
                w_T_a(1,1) = cos_t;
                w_T_a(0,2) = P1[0];
                w_T_a(1,2) = P1[1];

                w_T_a(2,0) = T(0.0);
                w_T_a(2,1) = T(0.0);
                w_T_a(2,2) = T(1.0);
            }


            // Convert P2 to T2 ^w_T_a
            Eigen::Matrix<T,3,3> w_T_b;
            {
                T cos_t = cos( P2[2] );
                T sin_t = sin( P2[2] );
                w_T_b(0,0) = cos_t;
                w_T_b(0,1) = -sin_t;
                w_T_b(1,0) = sin_t;
                w_T_b(1,1) = cos_t;
                w_T_b(0,2) = P2[0];
                w_T_b(1,2) = P2[1];

                w_T_b(2,0) = T(0.0);
                w_T_b(2,1) = T(0.0);
                w_T_b(2,2) = T(1.0);
            }

            Eigen::Matrix<T,3,3> w_B_L;


            // cast from double to T
            Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
            Eigen::Matrix<T, 3, 3> T_base_Tcap_laser;

#if 0
            T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                    T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                    T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));
#endif


            T_a_Tcap_b = a_Tcap_b.template cast<T>();
            T_base_Tcap_laser = base_Tcap_laser.template cast<T>();



            // now we have :: w_T_a, w_T_b and a_Tcap_b
            // compute pose difference
//            Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

            Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (T_base_Tcap_laser*w_T_a * w_T_b.inverse()*T_base_Tcap_laser.inverse());


            residual[0] = diff(0,2)*weight_x;
            residual[1] = diff(1,2)*weight_y;
            residual[2] = ceres::atan2( diff(1,0),diff(0,0) )*weight_yaw;

//        std::cout << "a_Tcap_b:\n" << a_Tcap_b << "\n" ;
//        std::cout << "w_T_a:\n" << w_T_a << "\n" ;
//        std::cout << "w_T_b:\n" << w_T_b<< "\n" ;
//        std::cout << "diff:\n" << diff<< "\n" ;
//        std::cout << " residual[0]:\n" <<  residual[0] << "\n" ;
//        std::cout << " residual[1]:\n" <<  residual[1] << "\n" ;
//        std::cout << " residual[2]:\n" <<  residual[2] << "\n" ;

            return true;

        }

    };


    /* to solve parameter: marker pose, laser pose
     *
     *
     *
     *
     * cost function
     *
     *
     *
     *
     *
     *
     * */
    struct PoseFusion{


        transform::Transform2d base_to_laser_tf;

        std::deque<transform::Transform2d> odom_to_base_tf_vec;
        constexpr static size_t BUFFER_SIZE = 5;
//

        PoseFusion(){

        }

        template<typename ...Args>
        static CostFunction* create (Args ...args){


            return new ceres::NumericDiffCostFunction<PoseFusion, ceres::CENTRAL, 1, 3>(new PoseFusion(args...));

        }

        template <typename T>
        bool operator()(const T* const pose_inv_1, const T* const pose_inv_2,const T* const pose_inv_3,const T* const pose_inv_4,const T* const pose_inv_5, T* residual) const{


            // pose_inv_1
            // pose_1
            // base_1

            const T* pos_array[BUFFER_SIZE] ={pose_inv_1,pose_inv_2,pose_inv_3,pose_inv_4,pose_inv_5};

            std::array<T,7> matrix_array[BUFFER_SIZE];

            for(int i = 0 ; i < BUFFER_SIZE;i++){

                matrix_array[i][0] = pos_array[i][0]; //tx
                matrix_array[i][1]= pos_array[i][1];  //ty
                matrix_array[i][2] = pos_array[i][2];  //yaw

                matrix_array[i][3] =  cos(matrix_array[2]); //cos
                matrix_array[i][4] =  -sin(matrix_array[2]);  //-sin
                matrix_array[i][6]   = matrix_array[3];  // cos
                matrix_array[i][5] = -matrix_array[4];  // sin
//                matrix_array[6]   = matrix_array[3];

            }
            for(int i = 0 ; i < BUFFER_SIZE;i++){

                for(int j = i+1 ; j < BUFFER_SIZE;j++){

                    // compare pose

                }
            }



            return true;


        }
    };



    // interplator

    struct PointsToGrid {
        enum {
            DATA_DIMENSION = 1
        };

        static const int GRID_SIZE = 100;
        double data[GRID_SIZE + GRID_SIZE][GRID_SIZE + GRID_SIZE] = {{1.0}};

        double mean_x = 0.0;
        double mean_y = 0.0;
        double occupied_value = 0.0;
        double free_value = 1.0;
        double resolution_inv ;

        PointsToGrid(const std::vector<float> &points, double resolution = 0.01, double filter_radius = 0.5 , double filter_step = 0.1,double smooth_radius = 3, double smooth_weight = 0.4) {

            resolution_inv = 1.0/resolution;

            int K = filter_radius * resolution_inv;


            free_value = (smooth_radius + smooth_weight * (K - smooth_radius)) * filter_step;

            /*LET'S INITIALIZE CELL VALUES TO= 0*/
            for (int i = 0; i < GRID_SIZE + GRID_SIZE-1; ++i) {
                for (int j = 0; j < GRID_SIZE + GRID_SIZE-1; ++j) {
                    data[i][j]=free_value;
                }
            }

            // get mean x,y
            mean_x = 0.0, mean_y = 0.0;
            int N = 0.5 * points.size();
            if (N == 0) return;

            for (int i = 0; i < N; i++) {
                mean_x += points[i + i];
                mean_y += points[i + i + 1];

            }
            mean_x /= N;
            mean_y /= N;

            // points to index
            int row, col;
            int index[1000];

//            std::cout << "\n====check grid\n" << std::endl;
            for (int i = 0; i < N; i++) {
                row = int(std::round((points[i + i] - mean_x) * resolution_inv )   ) + GRID_SIZE;
                col = int(std::round((points[i + i + 1] - mean_y) * resolution_inv )  )  + GRID_SIZE;

//                std::cout << "[ " << points[i + i] << ", " << points[i + i + 1] << ", " << row << ", " << col << " ]\n";

                data[row][col] = occupied_value;
                data[row][col] = occupied_value;
#if 0
                data[row+1][col] = occupied_value;
            data[row-1][col] = occupied_value;
            data[row][col+1] = occupied_value;
            data[row][col-1] = occupied_value;
#endif

                index[i+i] = row;
                index[i+i + 1 ] = col;

            }


            int index_i, index_j;

            for (int i = 0; i < N; i++) {
                index_i = index[i+i];
                index_j = index[i+i + 1 ];

                int m1 = std::max(index_i-K, 0);
                int m2 = std::max(index_i+K, GRID_SIZE + GRID_SIZE-1);
                int n1 = std::max(index_j-K, 0);
                int n2 = std::max(index_j+K, GRID_SIZE + GRID_SIZE-1);
                for(int m = m1 ; m < m2; m++){
                    for(int n = n1; n < n2; n++){
                        double dist =  std::sqrt((m-index_i)*(m-index_i) + (n -index_j)*(n -index_j));

                        dist = (dist > smooth_radius) ?  smooth_radius + smooth_weight * (dist - smooth_radius): dist;

                        data[m][n]  = std::min(dist* filter_step ,  data[m][n] ) ;

                    }
                }

            }
        }

        void GetValue(const int r, const int c, double *f) const {
//            std::cout << "== GetValue : " <<   r << ", " << c << "\n";
            const int row_idx =
                    (std::min)((std::max)(0, r), GRID_SIZE + GRID_SIZE - 1);
            const int col_idx =
                    (std::min)((std::max)(0, c), GRID_SIZE + GRID_SIZE - 1);
//            std::cout << "== GetIndex : " <<   row_idx << ", " << col_idx << "\n";

            f[0] = static_cast<double>(data[row_idx][col_idx]);
        }

        template<typename T>
        void GetIndex(const T &r, const T &c, T &r_index, T &c_index) const {
            r_index = ((r - mean_x) * resolution_inv  )+ T(GRID_SIZE);
            c_index = ((c - mean_y) * resolution_inv  )+ T(GRID_SIZE);
        }

    };


    // Cost-function using autodiff interface of BiCubicInterpolator
    struct SimpleAutoDiffBiCubicCost {

        template<typename T>
        bool operator()(const T *P, T *residual) const {

            // transform points from laser frame to map frame

            T tx = P[0];
            T ty = P[1];

            T cos_t = cos(P[2]);
            T sin_t = sin(P[2]);

            T r00 = cos_t;
            T r01 = -sin_t;
            T r10 = sin_t;
            T r11 = cos_t;

            T dst_point[2];
            T dst_index[2];

            std::cout << "P   = " << P[0] << ", " <<  P[1] << ", " << P[2] << std::endl;

            int N = 0.5*points_.size();
            T value = T(0.0);
            for(int i = 0; i < N;i++){
                dst_point[0] = r00 * points_[i+i] + r01 * points_[i+i+1] + tx;
                dst_point[1] = r10 * points_[i+i] + r11 * points_[i+i+1] + ty;



                grid_->template GetIndex(dst_point[0],dst_point[1],dst_index[0],dst_index[1]);
                interpolator_->Evaluate(dst_index[0],dst_index[1], &(residual[i] ));


                std::cout << "points_   = " << points_[i+i] << ", " <<  points_[i+i+1] << std::endl;
                std::cout << "dst_point = " <<dst_point[0]  << ", " <<  dst_point[1]  << std::endl;
                std::cout << "dst_index = " <<dst_index[0]  << ", " <<  dst_index[1]  << std::endl;
                std::cout << "residual[i] = " <<residual[i]   << std::endl;


                residual[i] = scaling_factor_ * residual[i];
            }


            return true;
        }

        SimpleAutoDiffBiCubicCost(PointsToGrid *grid ,ceres::BiCubicInterpolator<PointsToGrid>* interpolator , double scaling_factor,
                                  const std::vector<float> &points )
                :   grid_(grid), interpolator_(interpolator) ,scaling_factor_(scaling_factor){

            common::simple_cast(points_, points);

        }

        static ceres::CostFunction *Create(PointsToGrid * grid ,ceres::BiCubicInterpolator<PointsToGrid>* interpolator , double scaling_factor,
                                           const std::vector<float> &points) {
            return new ceres::AutoDiffCostFunction<SimpleAutoDiffBiCubicCost, ceres::DYNAMIC, 3>(
                    new SimpleAutoDiffBiCubicCost(grid,interpolator, scaling_factor, points),  static_cast<int>(0.5*points.size()));
        }

        std::vector<double> points_;
        PointsToGrid * grid_;
        ceres::BiCubicInterpolator<PointsToGrid>* interpolator_;
        double scaling_factor_ = 1.0;

    };


// Cost-function using autodiff interface of BiCubicInterpolator
    struct SimpleAutoDiffBiCubicCost_V2 {

        template<typename T>
        bool operator()(const T *P, T *residual) const {

            // transform points from laser frame to map frame

            T tx = P[0];
            T ty = P[1];

            T cos_t = cos(P[2]);
            T sin_t = sin(P[2]);

            T r00 = cos_t;
            T r01 = -sin_t;
            T r10 = sin_t;
            T r11 = cos_t;

            T dst_point[2];
            T dst_index[2];


            int N = 0.5*points_.size();
            T value = T(0.0);
            for(int i = 0; i < N;i++){
                dst_point[0] = r00 * points_[i+i] + r01 * points_[i+i+1] + tx;
                dst_point[1] = r10 * points_[i+i] + r11 * points_[i+i+1] + ty;



                grid_->template GetIndex(dst_point[0],dst_point[1],dst_index[0],dst_index[1]);
                interpolator_.Evaluate(dst_index[0],dst_index[1], &(residual[i] ));


//            std::cout << "points_   = " << points_[i+i] << ", " <<  points_[i+i+1] << std::endl;
//            std::cout << "dst_point = " <<dst_point[0]  << ", " <<  dst_point[1]  << std::endl;
//            std::cout << "dst_index = " <<dst_index[0]  << ", " <<  dst_index[1]  << std::endl;
//            std::cout << "residual[i] = " <<residual[i]   << std::endl;


                residual[i] = scaling_factor_ * residual[i];
            }


            return true;
        }

        SimpleAutoDiffBiCubicCost_V2(PointsToGrid *grid ,double scaling_factor,
                                     const std::vector<float> &points )
                :   grid_(grid), interpolator_(*grid) ,scaling_factor_(scaling_factor){

            common::simple_cast(points_, points);

        }

        static ceres::CostFunction *Create(PointsToGrid * grid , double scaling_factor,
                                           const std::vector<float> &points) {
            return new ceres::AutoDiffCostFunction<SimpleAutoDiffBiCubicCost_V2, ceres::DYNAMIC, 3>(
                    new SimpleAutoDiffBiCubicCost_V2(grid, scaling_factor, points),  static_cast<int>(0.5*points.size()));
        }

        std::vector<double> points_;
        PointsToGrid * grid_;
        const ceres::BiCubicInterpolator<PointsToGrid> interpolator_;
        double scaling_factor_ = 1.0;

    };
}



#endif //SCAN_REPUBLISHER_CERES_SCAN_TO_MARKER_H
