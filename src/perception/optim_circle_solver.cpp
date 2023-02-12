//
// Created by waxz on 23-2-8.
//

#include "optim_circle_solver.h"
#include "math/math_basic.h"

#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>


namespace perception{

    autodiff::var
    circle_opt_fnd(const autodiff::ArrayXvar& x, const std::vector<geometry::Point>& points,int point_num, float radius_2)
    {
        autodiff::var r = 0.0;
        autodiff::var t = 0.0;

        for(int i = 0 ; i < point_num ;i++){
            t = (x(0) - points[i].x)*(x(0) - points[i].x) + (x(1) - points[i].y)*(x(1) - points[i].y) - radius_2;
            r += t*t;
        }
        return r;
    }

    struct CircleCostFunction{

        const std::vector<geometry::Point>& points;
        int point_num;
        float radius = 0.1;
        float radius_2 = radius;


        CircleCostFunction(const std::vector<geometry::Point>& t_points, int t_point_num, float t_radius):points(t_points),point_num(t_point_num), radius(t_radius),radius_2(radius*radius){
        }



        double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
        {

            autodiff::ArrayXvar xd = x.eval();

            autodiff::var y = circle_opt_fnd(xd,points,point_num,radius_2);

            if (grad_out) {
                Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

                *grad_out = grad_tmp;
            }

            return autodiff::val(y);
        }
    };



    // find circle
    void FindCircle(std::vector<geometry::Point>& points,float radius, float edge_radius, float edge_range_offset, int min_point_num, float & center_x, float & center_y, float& error_mean){

        center_x = 100.0;
        center_y = 100.0;
        error_mean = 100.0;

        if(points.size() < min_point_num){

            return;
        }


        float angle_mean = std::atan2(points[ int(0.5*points.size()) ].y,points[ int(0.5*points.size())].x ),angle_sum = 0.0;




        int valid_num = 0;
        float min_r = points[0].r, max_r = points[0].r + edge_radius;

//    std::cout << "compute center angle:\n";
        float normalise_angle = 0.0;
        float edge_range = 0.0;

        for(auto&p : points){
            normalise_angle = std::abs(p.b - angle_mean) < M_PI ? p.b : (p.b + ((p.b - angle_mean) > 0.0 ? -M_PI*2: M_PI*2) );
//        std::cout << "angle: " << p.b <<", normalise_angle: " << normalise_angle << "\n";

            angle_sum += normalise_angle;
            edge_range += p.r;
            valid_num++;

        }

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

        if(valid_num < min_point_num){
            return;
        }


        angle_mean = angle_sum/float(valid_num);
        edge_range /= float(valid_num);

        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  angle_mean " << angle_mean << std::endl;  ;




        float ux = std::cos(angle_mean);
        float uy = std::sin(angle_mean);


        float circle_range =  edge_range + radius;


        float cx = ux*(circle_range);
        float cy = uy*(circle_range);

        center_x = ux*(circle_range);
        center_y = uy*(circle_range);
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " edge_range " << edge_range << ", circle_range " << circle_range << std::endl;  ;
        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " center: [" << center_x << ", " <<center_y << " ]" << std::endl;  ;


//    cx = points[0].x + ux*(radius);
//    cy = points[0].y + uy*(radius);

        float error_sum = 0.0 ;


        error_sum = 0.0;
//    std::cout << "check error:\n";
        for(int i = 0 ; i <points.size();i++){
            auto& p = points[i];
            float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
//        std::cout << d << ", ";
            error_sum += std::abs(d- radius) ;
        }
        error_mean = error_sum/float(points.size());


        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  init error_mean " << error_mean << std::endl;  ;



        Eigen::VectorXd x(2);
        x << cx, cy;

        optim::algo_settings_t settings;
//    settings.iter_max = 20;
//    settings.bfgs_settings.wolfe_cons_1 = 1e-4;
//    settings.bfgs_settings.wolfe_cons_2 = 0.8;

//    settings.print_level = 1;


        settings.vals_bound = true;

        settings.lower_bounds = optim::ColVec_t::Zero(2);
        settings.lower_bounds(0) = cx - 0.1;
        settings.lower_bounds(1) = cy - 0.1;

        settings.upper_bounds = optim::ColVec_t::Zero(2);
        settings.upper_bounds(0) = cx + 0.1;
        settings.upper_bounds(1) = cy + 0.1;

        CircleCostFunction opt_fn_obj(points,valid_num, radius) ;

        bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);

        if (success) {
            std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
        } else {
            std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
        }

        std::cout << "solution: x = \n" << x << std::endl;

        cx = x(0);
        cy = x(1);

        error_sum = 0.0;
        for(int i = 0 ; i <points.size();i++){
            auto& p = points[i];
            float d = std::sqrt( (p.x - cx)*(p.x  -cx) + (p.y - cy)*(p.y - cy)  );
//        std::cout << d << ", ";
            error_sum += std::abs(d- radius) ;
        }
        error_mean = error_sum/(float(points.size()) * (points.back().b - points.front().b ) );


        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  optim points.back().b - points.front().b " << (points.back().b - points.front().b) << std::endl;  ;

        std::cout << __FILE__ << ":" << __LINE__ << " @" << __FUNCTION__  << " check  optim error_mean " << error_mean << std::endl;  ;

        center_x = cx;
        center_y = cy;
        return  ;



    }

}