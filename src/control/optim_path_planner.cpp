//
// Created by waxz on 23-2-15.
//


#include "transform/transform.h"
#include "control_utils.h"
#include "optim_path_planner.h"


#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

//#include <autodiff/forward/real.hpp>
//#include <autodiff/forward/real/eigen.hpp>

#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>

namespace control{
    autodiff::var
    curve_opt_fnd(const autodiff::ArrayXvar& x,  const transform::Transform2d& current_pose, const transform::Transform2d& target_pose,int curve_num , const std::vector<float>& weight)
    {
        autodiff::var r = 0.0;

        transform::MatrixSE2<autodiff::var> init_pose(current_pose );
        std::vector<transform::MatrixSE2<autodiff::var> > all_next_pose(curve_num);

        transform::MatrixSE2<autodiff::var> next_pose = init_pose;
        for(int i = 0 ; i < curve_num; i++){
            next_pose = all_next_pose[i] = updateCurveDistDiff(next_pose, x(i+i),x(i+i+1));

        }

        auto& first_pose = all_next_pose.front();
        auto& final_pose = all_next_pose.back();

        autodiff::var control_first_angle = atan2( -first_pose.y(), -first_pose.x() );


        autodiff::var dist_error = (final_pose.x() - target_pose.x() )*(final_pose.x() - target_pose.x() ) + (final_pose.y() - target_pose.y() )*(final_pose.y() - target_pose.y() ) ;

        autodiff::var angle_error = (final_pose.yaw() - target_pose.yaw() )*(final_pose.yaw() - target_pose.yaw() );

        autodiff::var curve_error_last = x(curve_num+curve_num-2)*x(curve_num+curve_num-2);
        autodiff::var roate_angle_1 = abs(x(0)*x(1));



//    transform::Transform2d next_pose = updateCurveDIst(current_pose, x(0),x(1));
        r =  dist_error*weight[0]
             + angle_error*weight[1]
             + curve_error_last*weight[2]
//            + ( roate_angle_1 + roate_angle_2 + roate_angle_3)*weight[3]
//            + (next_pose_1.x() - 0.8*init_pose.x())*(next_pose_1.x() - 0.8*init_pose.x()) * weight[4]
//              + ( roate_angle_1 - control_first_angle)*( roate_angle_1 - control_first_angle)*weight[4]

//            + ( (x(5) < 0.2) ? (  (x(5) - 0.2) * (x(5) - 0.2) * weight[4] ) : (x(5) - 0.2) * (x(5) - 0.2)*0.0  )
//            + 0.1*control_pose.x() *control_pose.x()
                ;

        if( dist_error > 0.5* (init_pose.x()*init_pose.x()  + init_pose.y()*init_pose.y() ) ){

            r +=
                    ( roate_angle_1 - control_first_angle)*( roate_angle_1 - control_first_angle)*weight[3]
//                + x(0)*x(0)*weight[4]

                    ;

        }


//    std::cout << "check dist_error x " << next_pose.x() - target_pose.x() << "\n";
//    std::cout << "check dist_error y " << next_pose.y() - target_pose.y() << "\n";
//    std::cout << "check dist_error yaw" << next_pose.yaw() - target_pose.yaw() << "\n";

//    std::cout << "check error " << dist_error << ", " << angle_error << ", " << final_curve  << "\n";
//    std::cout << "check error_sum " << r  << "\n";


        return r;
    }

    struct CurveCostFunction{
        int curve_num = 3;
        transform::Transform2d current_pose;
        transform::Transform2d target_pose;
        std::vector<float> weight;
        CurveCostFunction(const transform::Transform2d& t_current_poose, const transform::Transform2d & t_target_pose ):current_pose(t_current_poose), target_pose(t_target_pose){


        }

        double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
        {

            autodiff::ArrayXvar xd = x.eval();

            autodiff::var y = curve_opt_fnd(xd,current_pose,target_pose,curve_num, weight);

            if (grad_out) {
                Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

                *grad_out = grad_tmp;
            }

            return autodiff::val(y);
        }

    };

    autodiff::var
    local_curve_opt_fnd(const autodiff::ArrayXvar& x,  const transform::Transform2d& current_pose, const transform::Transform2d& target_pose,float current_curve  , const std::vector<float>& weight,const std::vector<float>& tolerance)
    {
        autodiff::var r = 0.0;

        transform::MatrixSE2<autodiff::var> init_pose(current_pose );
        auto final_pose = updateCurveDistDiff(init_pose, x(0),x(1));


        autodiff::var final_dist_error_x =  abs(final_pose.x() - target_pose.x());
        autodiff::var final_dist_error_y =  abs(final_pose.y() - target_pose.y());
        autodiff::var final_dist_error_yaw =  abs(final_pose.yaw() - target_pose.yaw());

        final_dist_error_x =  final_dist_error_x < tolerance[0]? final_dist_error_x*weight[4] :final_dist_error_x*weight[0];
        final_dist_error_y =  final_dist_error_y < tolerance[1]? final_dist_error_y*weight[4] :final_dist_error_y*weight[1];
        final_dist_error_yaw =  final_dist_error_yaw < tolerance[2]? final_dist_error_yaw*weight[4] :final_dist_error_yaw*weight[2];


//    transform::Transform2d next_pose = updateCurveDIst(current_pose, x(0),x(1));
        r =  final_dist_error_x
                + final_dist_error_y
                  + final_dist_error_yaw
                  + (x(0) - current_curve)*(x(0) - current_curve) *weight[3]
                ;



//    std::cout << "check dist_error x " << next_pose.x() - target_pose.x() << "\n";
//    std::cout << "check dist_error y " << next_pose.y() - target_pose.y() << "\n";
//    std::cout << "check dist_error yaw" << next_pose.yaw() - target_pose.yaw() << "\n";

//    std::cout << "check error " << dist_error << ", " << angle_error << ", " << final_curve  << "\n";
//    std::cout << "check error_sum " << r  << "\n";


        return r;
    }

    struct LocalCurveCostFunction{
        int curve_num = 3;
        transform::Transform2d current_pose;
        transform::Transform2d target_pose;
        std::vector<float> weight;
        std::vector<float> tolerance;
        float current_curve = 0.0;


        LocalCurveCostFunction(const transform::Transform2d& t_current_poose, const transform::Transform2d & t_target_pose ):current_pose(t_current_poose), target_pose(t_target_pose){


        }

        double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
        {

            autodiff::ArrayXvar xd = x.eval();

            autodiff::var y = local_curve_opt_fnd(xd,current_pose,target_pose,current_curve, weight,tolerance);

            if (grad_out) {
                Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

                *grad_out = grad_tmp;
            }

            return autodiff::val(y);
        }

    };

    bool PathPlanner::solve(const transform::Transform2d& robot_init_pose, const transform::Transform2d& target_pose){

        float optim_dist = sqrt((robot_init_pose.x() -target_pose.x()) *(robot_init_pose.x() -target_pose.x())
                                + (robot_init_pose.y() -target_pose.y())*(robot_init_pose.y() -target_pose.y())  );
        size_t optim_param_num = 2*curve_num;
        float optim_dist_step = optim_dist/(float(curve_num));

        Eigen::VectorXd x(optim_param_num);
        for(int i = 0 ; i < curve_num;i++){
            x(i+i) = 1e-5;
            x(i+i+1) = optim_dist_step;

        }

        optim::algo_settings_t settings;
        {
            //nan
            settings.iter_max = 20;
            settings.bfgs_settings.wolfe_cons_1 = 1e-3;
            settings.bfgs_settings.wolfe_cons_2 = 0.9;

            settings.conv_failure_switch = 1;

        }

        settings.print_level = 1;

        settings.vals_bound = true;

        settings.lower_bounds = optim::ColVec_t::Zero(optim_param_num);
        settings.upper_bounds = optim::ColVec_t::Zero(optim_param_num);

        for(int i = 0 ; i < curve_num;i++){
            settings.lower_bounds(i+i) = -100.0;
            settings.lower_bounds(i+i+1) = 0.0;
            settings.upper_bounds(i+i) = 100.0;
            settings.upper_bounds(i+i +1) = 1.0;
        }



        CurveCostFunction opt_fn_obj(robot_init_pose,target_pose) ;

        {
            //nan
            opt_fn_obj.weight = std::vector<float>{0.1, 0.1, 0.02, 0.01, 0.002, 0.002};
            opt_fn_obj.curve_num = curve_num;

        }

        bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);

        if (success) {
            std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
        } else {
            std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
        }

        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " solution: x = \n" << x << std::endl;

        if(!std::isnormal(x.norm())){
            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  <<   "optim get nan value" << x << std::endl;
            params_compute_ok = false;
            return false;
        }

        path_segments.resize(curve_num);
        transform::Transform2d next_pose = robot_init_pose;
        for(int i = 0 ; i < curve_num;i++){
            interpolatePath(next_pose,x(i+i) ,x(i+i+1),10, path_segments[i].path);
            path_segments[i].curve = x(i+i);
            path_segments[i].dist = x(i+i+1);
            path_segments[i].current_id = 0;
            next_pose =  path_segments[i].path.back();
        }

        next_pose = robot_init_pose;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  <<  " : robot_init_pose:= \n" <<robot_init_pose << std::endl;

        for(int i = 0 ; i < curve_num;i++){
            next_pose =  updateCurveDist(next_pose, x(i+i) ,x(i+i+1));
            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", curve = " << x(i+i) << ", dist = " << x(i+i+1) <<" , next_pose:= \n" <<next_pose << std::endl;

        }

        params_compute_ok =
                std::fabs(next_pose.x() - target_pose.x()) < x_tolerance
                && std::fabs(next_pose.y() - target_pose.y()) < y_tolerance
                && std::fabs(next_pose.yaw() - target_pose.yaw()) < yaw_tolerance
                ;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", params_compute_ok : " << params_compute_ok <<  " : final pose:= \n" <<next_pose << std::endl;

        if(params_compute_ok){
            solved_params = x;

        }
        return params_compute_ok;


    }

    bool PathPlanner::computeLocalCurve(const transform::Transform2d& robot_init_pose, float& curve, float& dist){
        if(!params_compute_ok || path_segments.empty() || path_segments.front().path.empty())
        {
            params_compute_ok = false;
            return false;

        }



        // find the closest pose
        // find local curve target pose
        float best_dist = 100.0;
        size_t best_segment_id = 0;
        size_t best_pose_id = 0;
        float min_segment_len = 0.05*0.05;


        for(size_t i = 0 ; i < path_segments.size();i++){

            auto& path = path_segments[i];
            for(size_t j = 0 ; j < path.path.size(); j++){
                auto& p = path.path[j];
                float dist = (p.x() - robot_init_pose.x())*(p.x() - robot_init_pose.x())
                        +(p.y() - robot_init_pose.y())*(p.y() - robot_init_pose.y());

                if(dist < best_dist){
                    best_dist = dist;
                    best_segment_id = i;
                    best_pose_id = j;
                }
            }
        }

        {

            auto& path = path_segments[best_segment_id];
            auto& p = path.path.back();
            float dist = (p.x() - robot_init_pose.x())*(p.x() - robot_init_pose.x())
                         +(p.y() - robot_init_pose.y())*(p.y() - robot_init_pose.y());

            if(dist < min_segment_len){

                if(best_segment_id < (path_segments.size() -1)){
                    best_segment_id +=1;

                }

            }


        }



        auto& local_path = path_segments[best_segment_id];

        auto& target_pose = local_path.path.back();


        float optim_dist = sqrt((robot_init_pose.x() - target_pose.x()) *(robot_init_pose.x() -target_pose.x())
                                + (robot_init_pose.y() -target_pose.y())*(robot_init_pose.y() -target_pose.y())  );



        Eigen::VectorXd x(2);
        x(0) = local_path.curve;
        x(1) = optim_dist;



        optim::algo_settings_t settings;
        {
            //nan
            settings.iter_max = 20;
            settings.bfgs_settings.wolfe_cons_1 = 1e-3;
            settings.bfgs_settings.wolfe_cons_2 = 0.9;

//            settings.conv_failure_switch = 1;

        }

        settings.print_level = 1;

        settings.vals_bound = true;

        settings.lower_bounds = optim::ColVec_t::Zero(2);
        settings.upper_bounds = optim::ColVec_t::Zero(2);

        settings.lower_bounds(0) = -100.0;
        settings.upper_bounds(0) = 100.0;

        settings.lower_bounds(1) = 0.0;
        settings.upper_bounds(1) = optim_dist + 0.3;


        LocalCurveCostFunction opt_fn_obj(robot_init_pose,target_pose) ;

        {
            //nan
            opt_fn_obj.weight = std::vector<float>{0.1, 0.1, 0.1, 0.0001, 0.002, 0.002};
            opt_fn_obj.tolerance = std::vector<float>{x_tolerance, y_tolerance, yaw_tolerance};

            opt_fn_obj.current_curve = local_path.curve;



        }

        bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);

        if (success) {
            std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
        } else {
            std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
        }

        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " solution: x = \n" << x << std::endl;

        if(!std::isnormal(x.norm())){
            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__   <<   "optim get nan value" << x << std::endl;
            params_compute_ok = false;
            return false;
        }
        curve = x(0);

        dist = x(1);

        transform::Transform2d next_pose = robot_init_pose;

        next_pose = robot_init_pose;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  <<  " : robot_init_pose:= \n" <<robot_init_pose << std::endl;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  <<  " : target_pose:= \n" <<target_pose << std::endl;

        for(int i = 0 ; i < 1;i++){
            next_pose =  updateCurveDist(next_pose, x(i+i) ,x(i+i+1));
            std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", curve = " << x(i+i) << ", dist = " << x(i+i+1) <<" , next_pose:= \n" <<next_pose << std::endl;

        }



//
//        interpolatePath(next_pose,x(0) ,x(1),10, local_path_segment.path);
//        local_path_segment.curve = x(0);
//        local_path_segment.dist = x(1);
//        local_path_segment.current_id = 0;
//
//
//        next_pose =  local_path_segment.path.back();

        local_params_compute_ok =
                std::fabs(next_pose.x() - target_pose.x()) < x_tolerance
                && std::fabs(next_pose.y() - target_pose.y()) < y_tolerance
                && std::fabs(next_pose.yaw() - target_pose.yaw()) < yaw_tolerance
                ;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << ", local_params_compute_ok : " << local_params_compute_ok <<  " : final pose:= \n" <<next_pose << std::endl;



        return local_params_compute_ok;


    }




}

