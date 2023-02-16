//
// Created by waxz on 23-1-4.
//
#define SOL_CHECK_ARGUMENTS 1
#include <sol.hpp>
#include <fstream>


#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "xmlrpcpp/XmlRpc.h"

//#include <matplot/matplot.h>


#include "nlohmann/json.hpp"
#include <plog/Log.h> // Step1: include the headers
#include "plog/Initializers/RollingFileInitializer.h"
#include "plog/Appenders/ColorConsoleAppender.h"


#include "common/task.h"
#include "common/suspend.h"
#include "common/clock_time.h"
#include "common/subprocess.hpp"



#include "sensor/laser_scan.h"
#include "sensor/geometry_types.h"
#include "sensor/pointcloud_ros.h"

#include "perception/pointcloud_detection.h"
#include "perception/laserscan_filter.h"
#include "perception/optim_circle_solver.h"
#include "perception/pcl_circle_fitting.h"

#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"

//#include <autodiff/forward/real.hpp>
//#include <autodiff/forward/real/eigen.hpp>

#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>
/*


 */

#include "pid.h"
#include "transform/pose_filter.h"



#include "control/optim_path_planner.h"
/*

 robot pose define in final target frame


                      |
                      |
                      |
                      |
              --------|t0

                      .t1

                      .t2


                _|r




  control parameter: curve_1, dist_1, cure_2, dist_2
  input: current_pose, current_vel, target_pose

 1. given current_vel, compute curve limit,
 2. constrains:






 */


/*
 update pose use curve and dist

 curve >0 : turn left
 curve <0 : turn right
 curve =0 : forward

 radius = 1.0/curve
 angle = dist/radius = dist*curve


 rotate_center_relative : transform(0.0, - radius, 0.0)
 rotate_center_absolute:  init_pose * rotate_center_relative
 next_pose_relative: transform(radius*cos(angle), radius*sin(angle), angle)
 next_pose_absolute : rotate_center_absolute*next_pose_relative

  */


transform::Transform2d updateCurveDIst(const transform::Transform2d& init_pose, float curve, float dist){
    transform::Transform2d result_pose;

    if(std::abs(curve) < 1e-4){

        result_pose.set(dist, 0.0,0.0);
        result_pose = init_pose *result_pose;
    }else{
        float radius = 1.0/curve;
        float angle = dist* curve;

        transform::Transform2d rotate_center_relative(0.0, radius, 0.0);
        transform::Transform2d rotate_center_absolute = init_pose*rotate_center_relative;
        float angle_relative = (curve>0.0) ? (angle - M_PI_2) : ( M_PI_2 + angle);

        float abs_radius = std::abs(radius);
        transform::Transform2d result_pose_relative(abs_radius*std::cos(angle_relative), abs_radius*std::sin(angle_relative), angle);

        result_pose = rotate_center_absolute * result_pose_relative;
    }
    return result_pose;
}


template<typename FloatType>
transform::MatrixSE2<FloatType> updateCurveDistDiff(const transform::MatrixSE2<FloatType>& init_pose, FloatType curve, FloatType dist){
    transform::MatrixSE2<FloatType> result_pose;

    {
        FloatType radius = 1.0/(curve);
        FloatType angle = dist* curve;

        transform::MatrixSE2<FloatType> rotate_center_relative(0.0, radius, 0.0);
        transform::MatrixSE2<FloatType> rotate_center_absolute = init_pose*rotate_center_relative;
        FloatType angle_relative = (curve>0.0) ? (angle - M_PI_2) : ( M_PI_2 + angle);

        FloatType abs_radius = abs(radius);
        transform::MatrixSE2<FloatType> result_pose_relative(abs_radius*cos(angle_relative), abs_radius*sin(angle_relative), angle);

        result_pose = rotate_center_absolute * result_pose_relative;
    }
    return result_pose;
}



autodiff::var
curve_opt_fnd2(const autodiff::ArrayXvar& x,  const transform::Transform2d& current_pose, const transform::Transform2d& target_pose,const std::vector<float>& weight)
{
    autodiff::var r = 0.0;

    transform::MatrixSE2<autodiff::var> init_pose(current_pose );

    transform::MatrixSE2<autodiff::var>next_pose = updateCurveDistDiff(init_pose, x(0),x(1));

    transform::MatrixSE2<autodiff::var> max_x_pose;

    for(float i = 0.1 ; i < 1.0 ; i+= 0.1){
        autodiff::var tmp_ratio = x(1) * i;
        transform::MatrixSE2<autodiff::var>next_pose_test = updateCurveDistDiff(init_pose, x(0),tmp_ratio);

        if(abs(next_pose_test.x()) > abs(max_x_pose.x()) ){
            max_x_pose = next_pose_test;
        }
    }

//    autodiff::var x1 = x(1) * 0.5;
//    transform::MatrixSE2<autodiff::var>control_pose = updateCurveDistDiff(init_pose, x(0), x1 );

//    next_pose = updateCurveDistDiff(init_pose, x(2),x(3));
    next_pose = updateCurveDistDiff(next_pose, x(2),x(3));
    next_pose = updateCurveDistDiff(next_pose, x(4),x(5));


    autodiff::var d = (next_pose.x() - target_pose.x() )*(next_pose.x() - target_pose.x() ) + (next_pose.y() - target_pose.y() )*(next_pose.y() - target_pose.y() ) ;
#if 0
    if(d < weight[0]){
        d *= weight[1];
    }else{
        d = weight[0]*weight[1] + (d - weight[0] );
    }

#endif
//    transform::Transform2d next_pose = updateCurveDIst(current_pose, x(0),x(1));
    r =  (d)*weight[2];
//            + (x(4)*x(4))*weight[4]
//            + ( x(0)*x(1)*x(0)*x(1) + x(2)*x(3)*x(2)*x(3) + x(4)*x(5)*x(4)*x(5))*weight[3]
//            + ( (x(5) < 0.2) ? (  (x(5) - 0.2) * (x(5) - 0.2) * weight[4] ) : (x(5) - 0.2) * (x(5) - 0.2)*0.0  )
//            + 0.1*control_pose.x() *control_pose.x()
    ;
    if(d < 0.04  ){
        r += (next_pose.yaw() - target_pose.yaw() )*(next_pose.yaw() - target_pose.yaw() )*weight[3]
             + (x(4)*x(4))*weight[4];
    }
#if 0
    if(d < 0.01 && ( (abs(max_x_pose.x()) - abs(init_pose.x()) ) > 0.1  ) ){
        r += abs(max_x_pose.x()) * weight[5];
    }
#endif
    return r;
}


autodiff::var
curve_opt_fnd(const autodiff::ArrayXvar& x,  const transform::Transform2d& current_pose, const transform::Transform2d& target_pose,const std::vector<float>& weight)
{
    autodiff::var r = 0.0;

    transform::MatrixSE2<autodiff::var> init_pose(current_pose );

    transform::MatrixSE2<autodiff::var> next_pose_1 = updateCurveDistDiff(init_pose, x(0),x(1));
    autodiff::var control_first_angle = atan2( -next_pose_1.y(), -next_pose_1.x() );

//    std::cout << "check x " << x << "\n";
//    std::cout << "check next_pose 1 " << next_pose.x() << ", " << next_pose.y() << ", " << next_pose.yaw() << "\n";

//    autodiff::var x1 = x(1) * 0.5;
//    transform::MatrixSE2<autodiff::var>control_pose = updateCurveDistDiff(init_pose, x(0), x1 );

//    next_pose = updateCurveDistDiff(init_pose, x(2),x(3));
    transform::MatrixSE2<autodiff::var>  next_pose = updateCurveDistDiff(next_pose_1, x(2),x(3));
//    std::cout << "check next_pose 2 " << next_pose.x() << ", " << next_pose.y() << ", " << next_pose.yaw() << "\n";

    next_pose = updateCurveDistDiff(next_pose, x(4),x(5));
//    std::cout << "check next_pose 3 " << next_pose.x() << ", " << next_pose.y() << ", " << next_pose.yaw() << "\n";

    autodiff::var dist_error = (next_pose.x() - target_pose.x() )*(next_pose.x() - target_pose.x() ) + (next_pose.y() - target_pose.y() )*(next_pose.y() - target_pose.y() ) ;

    autodiff::var angle_error = (next_pose.yaw() - target_pose.yaw() )*(next_pose.yaw() - target_pose.yaw() );

    autodiff::var final_curve = x(4)*x(4);
    autodiff::var roate_angle_1 = abs(x(0)*x(1));
    autodiff::var roate_angle_2 = abs(x(2)*x(3));
    autodiff::var roate_angle_3 = abs(x(4)*x(5));



//    transform::Transform2d next_pose = updateCurveDIst(current_pose, x(0),x(1));
    r =  dist_error*weight[0]
            + angle_error*weight[1]
            + final_curve*weight[2]
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
    transform::Transform2d current_pose;
    transform::Transform2d target_pose;
    std::vector<float> weight;
    CurveCostFunction(const transform::Transform2d& t_current_poose, const transform::Transform2d & t_target_pose ):current_pose(t_current_poose), target_pose(t_target_pose){


    }

    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
    {

        autodiff::ArrayXvar xd = x.eval();

        autodiff::var y = curve_opt_fnd(xd,current_pose,target_pose,weight);

        if (grad_out) {
            Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

            *grad_out = grad_tmp;
        }

        return autodiff::val(y);
    }

};











//float NormaliseAngle(float angle, float mean){
//    while( std::abs(angle - mean) > M_PI)
//}




#if 0
// The multi-variable function for which derivatives are needed
autodiff::real f(autodiff::real x, autodiff::real y, autodiff::real z)
{
    return 1 + x + y + z + x*y + y*z + x*z + x*y*z + exp(x/y + y/z);
}
// The multi-variable function for which derivatives are needed
autodiff::var f2(autodiff::var x, autodiff::var y, autodiff::var z)
{
    return 1 + x + y + z + x*y + y*z + x*z + x*y*z + exp(x/y + y/z);
}
#endif

autodiff::var
opt_fnd(const autodiff::ArrayXvar& x)
{
    return (x(0) - 1.0)*(x(0) - 1.0)
    + (x(1) - 1.5)*(x(1) - 1.5)
      + (x(2) - 1.8)*(x(2) - 1.8)
    ;
}


double
opt_fn(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
{

    autodiff::ArrayXvar xd = x.eval();

    autodiff::var y = opt_fnd(xd);

    if (grad_out) {
        Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

        *grad_out = grad_tmp;
    }

    return autodiff::val(y);
}



void FindShelfLeg(){}


/*

 todo:
 1. center/radius filter, update with odom change
 2.

 */


/*
 detect target:
 1. one circle
 2. 4 legs of a shelf
 3. 2 legs of a shelf
 */



/*

 compute control points array, not direct rotation and forward cmd_vel
 two stage controller
 1.

 */
struct MotionControl{

};



//https://sol2.readthedocs.io/en/v2.20.6/exceptions.html#lua-handlers

int my_exception_handler(lua_State* L, sol::optional<const std::exception&> maybe_exception, sol::string_view description) {
    // L is the lua state, which you can wrap in a state_view if necessary
    // maybe_exception will contain exception, if it exists
    // description will either be the what() of the exception or a description saying that we hit the general-case catch(...)
    std::cout << "An exception occurred in a function, here's what it says ";
    if (maybe_exception) {
        std::cout << "(straight from the exception): ";
        const std::exception& ex = *maybe_exception;
        std::cout << ex.what() << std::endl;
    }
    else {
        std::cout << "(from the description parameter): ";
        std::cout.write(description.data(), description.size());
        std::cout << std::endl;
    }

    // you must push 1 element onto the stack to be
    // transported through as the error object in Lua
    // note that Lua -- and 99.5% of all Lua users and libraries -- expects a string
    // so we push a single string (in our case, the description of the error)
    return sol::stack::push(L, description);
}
#if 0
void plotTransform2d(const transform::Transform2d& pose,const char* color = "black", float length = 0.03){

    auto a = matplot::arrow(pose.x(), pose.y(), pose.x() + length*std::cos(pose.yaw()),pose.y() + length*std::sin(pose.yaw()));

    a->color(color);
}

void test_updateCurveDIst(){
    transform::Transform2d pose(0.5,0.2,-0.1);
    float radius = 0.3;
    float curve = 1.0/radius;
    int N = 30;
    float dist_inc = 2*M_PI*radius/float(N);
    float dist = 0.0;
    transform::Transform2d next_pose;




    auto a =matplot::arrow(pose.x(), pose.y(), pose.x() + 0.03*std::cos(pose.yaw()),pose.y() + 0.03*std::sin(pose.yaw()));

    PLOGW << "round 1" <<std::endl;
    a->color("red");
    curve = 1.0/radius;
    dist = 0.0;
    for(int i = 0 ; i < N; i++){
        next_pose = updateCurveDIst(pose, curve, dist);

        plotTransform2d(next_pose,"blue");
        char buffer[20];
        sprintf(buffer, "%i", i);
        matplot::text(next_pose.x(), next_pose.y(), buffer)->color("red").font_size(8);

//            PLOGD << "curve : " << curve << ", dist : " << dist << ", next_pose : " << next_pose << "\n";
        dist += dist_inc;
    }

    for(int i = 0 ; i < N; i++){
        next_pose = updateCurveDIst(next_pose, 0.0, 0.1);
        plotTransform2d(next_pose,"blue");

        char buffer[20];
        sprintf(buffer, "%i", i);
        matplot::text(next_pose.x(), next_pose.y(), buffer)->color("red").font_size(8);

//            PLOGD << "curve : " << curve << ", dist : " << dist << ", next_pose : " << next_pose << "\n";
        dist += dist_inc;
    }

    PLOGW << "round 2" <<std::endl;

    pose = next_pose;
    curve = -1.0/radius;
    dist = 0.0;

    for(int i = 0 ; i < N; i++){
        next_pose = updateCurveDIst(pose, curve, dist);
        plotTransform2d(next_pose,"blue");


        char buffer[20];
        sprintf(buffer, "%i", i);
        matplot::text(next_pose.x(), next_pose.y(), buffer)->color("red").font_size(8);

//            PLOGD << "curve : " << curve << ", dist : " << dist << ", next_pose : " << next_pose << "\n";
        dist += dist_inc;
    }


    dist = 0.0;

    for(int i = 0 ; i < N/2; i++){
        next_pose = updateCurveDIst(next_pose, 0.0, -0.1);
        plotTransform2d(next_pose,"blue");

        char buffer[20];
        sprintf(buffer, "%i", i);
        matplot::text(next_pose.x(), next_pose.y(), buffer)->color("red").font_size(8);

//            PLOGD << "curve : " << curve << ", dist : " << dist << ", next_pose : " << next_pose << "\n";
        dist += dist_inc;
    }


    pose = next_pose;
    curve = -1.0/radius;
    dist = 0.0;

    for(int i = 0 ; i < N; i++){
        next_pose = updateCurveDIst(pose, curve, -dist);
        plotTransform2d(next_pose,"blue");

        char buffer[20];
        sprintf(buffer, "%i", i);
        matplot::text(next_pose.x(), next_pose.y(), buffer)->color("red").font_size(8);

//            PLOGD << "curve : " << curve << ", dist : " << dist << ", next_pose : " << next_pose << "\n";
        dist += dist_inc;
    }

    pose = next_pose;
    curve = 1.0/radius;
    dist = 0.0;

    for(int i = 0 ; i < N; i++){
        next_pose = updateCurveDIst(pose, curve, -dist);
        plotTransform2d(next_pose,"blue");


        char buffer[20];
        sprintf(buffer, "%i", i);
        matplot::text(next_pose.x(), next_pose.y(), buffer)->color("red").font_size(8);

//            PLOGD << "curve : " << curve << ", dist : " << dist << ", next_pose : " << next_pose << "\n";
        dist += dist_inc;
    }
    matplot::show();
}

void test_PathSolver(float init_x, float init_y, float init_yaw){


    float current_vel[2] = {0.1, 0.0};

    transform::Transform2d robot_init_pose(init_x,init_y,init_yaw);
    transform::Transform2d target_pose(0.0,0.0,0.0);
    plotTransform2d(robot_init_pose,"green");
    plotTransform2d(target_pose,"red");




    size_t optim_param_num = 6;
    Eigen::VectorXd x(optim_param_num);
    x << 1e-5, 0.1 ,1e-5,0.1,1e-5,0.1;

    optim::algo_settings_t settings;
//    settings.iter_max = 20;
//    settings.bfgs_settings.wolfe_cons_1 = 1e-4;
//    settings.bfgs_settings.wolfe_cons_2 = 0.8;

//    settings.print_level = 1;

    settings.vals_bound = true;

    settings.lower_bounds = optim::ColVec_t::Zero(optim_param_num);
    settings.lower_bounds(0) = -100.0;
    settings.lower_bounds(1) = 0.0;
    settings.lower_bounds(2) = -100.0;
    settings.lower_bounds(3) = 0.0;

    settings.lower_bounds(4) = -100.0;
    settings.lower_bounds(5) = 0.0;
    settings.upper_bounds = optim::ColVec_t::Zero(optim_param_num);
    settings.upper_bounds(0) = 100.0;
    settings.upper_bounds(1) = 1.0;
    settings.upper_bounds(2) = 100.0;
    settings.upper_bounds(3) = 1.0;

    settings.upper_bounds(4) = 100.0;
    settings.upper_bounds(5) = 1.0;
    CurveCostFunction opt_fn_obj(robot_init_pose,target_pose) ;

    opt_fn_obj.weight = std::vector<float>{0.02*0.02,0.1,1.0, 1.0, 0.2, 0.0001, 0.2, 0.2};
   opt_fn_obj.weight = std::vector<float>{1.0, 1.0, 0.2, 0.0001, 0.2, 0.2};
    bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "solution: x = \n" << x << std::endl;


    float curve = 0.0;
    float dist = 0.0;
    int N= 30;
    float dist_inc = x(1)/float(N);
    transform::Transform2d next_pose;

    dist = 0.0;
    curve= x(0);
    dist_inc = x(1)/float(N);
    for(int i = 0; i < N; i++){
        next_pose = updateCurveDIst(robot_init_pose, curve, dist);
        plotTransform2d(next_pose,"blue");
        dist += dist_inc;
    }
    robot_init_pose = updateCurveDIst(robot_init_pose, x(0), x(1));
    plotTransform2d(robot_init_pose,"red");
    dist = 0.0;
    curve= x(2);
    dist_inc = x(3)/float(N);

    for(int i = 0; i < N; i++){
        next_pose = updateCurveDIst(robot_init_pose, curve, dist);
        plotTransform2d(next_pose,"black");
        dist += dist_inc;
    }
    robot_init_pose = updateCurveDIst(robot_init_pose, x(2), x(3));
    plotTransform2d(robot_init_pose,"red");

    dist = 0.0;
    curve= x(4);
    dist_inc = x(5)/float(N);

    for(int i = 0; i < N; i++){
        next_pose = updateCurveDIst(robot_init_pose, curve, dist);
        plotTransform2d(next_pose,"blue");
        dist += dist_inc;
    }
    robot_init_pose = updateCurveDIst(robot_init_pose, x(4), x(5));
    plotTransform2d(robot_init_pose,"red");

    matplot::ylim({-2, +2});
    matplot::xlim({-3, +1});


}


void test_2(){

//        test_updateCurveDIst();
    test_PathSolver(-0.8, 0.1, 0.2);
    test_PathSolver(-0.6, 0.2, 0.2);

    test_PathSolver(-0.8, -0.1, 0.2);
    test_PathSolver(-0.6, -0.2, 0.2);


    test_PathSolver(-0.8, 0.1, -0.3);
    test_PathSolver(-0.6, 0.2, -0.3);

    test_PathSolver(-0.8, -0.1, -0.3);
    test_PathSolver(-0.6, -0.2, -0.3);

    test_PathSolver(-0.1, -0.05, -0.05);
    test_PathSolver(-0.2, -0.1, -0.1);


    matplot::show();
}
#endif


Eigen::VectorXd  test_PathSolver(float init_x, float init_y, float init_yaw){


    float current_vel[2] = {0.1, 0.0};

    transform::Transform2d robot_init_pose(init_x,init_y,init_yaw);
    transform::Transform2d target_pose(0.0,0.0,0.0);
//    plotTransform2d(robot_init_pose,"green");
//    plotTransform2d(target_pose,"red");




    float optim_dist = sqrt(init_x*init_x + init_y*init_y);
    size_t optim_param_num = 6;
    float optim_dist_step = 0.6*optim_dist/(0.5*float(optim_param_num));

    Eigen::VectorXd x(optim_param_num);

    x << 1e-5, optim_dist_step ,1e-5,optim_dist_step,1e-5,optim_dist_step;

    optim::algo_settings_t settings;
    settings.iter_max = 50;
    settings.bfgs_settings.wolfe_cons_1 = 1e-4;
    settings.bfgs_settings.wolfe_cons_2 = 0.8;


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
    settings.lower_bounds(0) = -100.0;
    settings.lower_bounds(1) = 0.0;
    settings.lower_bounds(2) = -100.0;
    settings.lower_bounds(3) = 0.0;

    settings.lower_bounds(4) = -100.0;
    settings.lower_bounds(5) = 0.0;
    settings.upper_bounds = optim::ColVec_t::Zero(optim_param_num);
    settings.upper_bounds(0) = 100.0;
    settings.upper_bounds(1) = 1.0;
    settings.upper_bounds(2) = 100.0;
    settings.upper_bounds(3) = 1.0;

    settings.upper_bounds(4) = 100.0;
    settings.upper_bounds(5) = 1.0;
    CurveCostFunction opt_fn_obj(robot_init_pose,target_pose) ;

    opt_fn_obj.weight = std::vector<float>{0.02*0.02,0.1,1.0, 1.0, 0.2, 0.0001, 0.2, 0.2};
    opt_fn_obj.weight = std::vector<float>{1.0, 1.0, 0.2, 0.0001, 0.001, 0.001};

    {
        //nan
        opt_fn_obj.weight = std::vector<float>{0.1, 0.1, 0.02, 0.01, 0.002, 0.002};

    }
    bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "solution: x = \n" << x << std::endl;


    float curve = 0.0;
    float dist = 0.0;
    int N= 30;
    float dist_inc = x(1)/float(N);
    transform::Transform2d next_pose;

    dist = 0.0;
    curve= x(0);
    dist_inc = x(1)/float(N);
    for(int i = 0; i < N; i++){
        next_pose = updateCurveDIst(robot_init_pose, curve, dist);
//        plotTransform2d(next_pose,"blue");
        dist += dist_inc;
    }
    robot_init_pose = updateCurveDIst(robot_init_pose, x(0), x(1));
//    plotTransform2d(robot_init_pose,"red");
    dist = 0.0;
    curve= x(2);
    dist_inc = x(3)/float(N);

    for(int i = 0; i < N; i++){
        next_pose = updateCurveDIst(robot_init_pose, curve, dist);
//        plotTransform2d(next_pose,"black");
        dist += dist_inc;
    }
    robot_init_pose = updateCurveDIst(robot_init_pose, x(2), x(3));
//    plotTransform2d(robot_init_pose,"red");

    dist = 0.0;
    curve= x(4);
    dist_inc = x(5)/float(N);

    for(int i = 0; i < N; i++){
        next_pose = updateCurveDIst(robot_init_pose, curve, dist);
//        plotTransform2d(next_pose,"blue");
        dist += dist_inc;
    }
    robot_init_pose = updateCurveDIst(robot_init_pose, x(4), x(5));
    PLOGD<<   "final robot_init_pose  = \n" << robot_init_pose << std::endl;
    if(!std::isnormal(x.norm())){
        PLOGD<<   "optim get nan value" << x << std::endl;

    }
    return x;
//    plotTransform2d(robot_init_pose,"red");

//    matplot::ylim({-2, +2});
//    matplot::xlim({-3, +1});


}


bool solve_curve_path_plan(const transform::Transform2d& robot_init_pose, const transform::Transform2d& target_pose,Eigen::VectorXd& optim_param){





    float optim_dist = sqrt((robot_init_pose.x() -target_pose.x()) *(robot_init_pose.x() -target_pose.x())
            + (robot_init_pose.y() -target_pose.y())*(robot_init_pose.y() -target_pose.y())  );
    size_t optim_param_num = 6;
    float optim_dist_step = 0.6*optim_dist/(0.5*float(optim_param_num));

    Eigen::VectorXd x(optim_param_num);

    x << 1e-5, optim_dist_step ,1e-5,optim_dist_step,1e-5,optim_dist_step;


    optim::algo_settings_t settings;
    settings.iter_max = 20;
    settings.bfgs_settings.wolfe_cons_1 = 1e-3;
    settings.bfgs_settings.wolfe_cons_2 = 0.9;

    settings.conv_failure_switch = 1;



    settings.print_level = 1;

    settings.vals_bound = true;

    settings.lower_bounds = optim::ColVec_t::Zero(optim_param_num);
    settings.lower_bounds(0) = -100.0;
    settings.lower_bounds(1) = 0.00;
    settings.lower_bounds(2) = -100.0;
    settings.lower_bounds(3) = 0.00;

    settings.lower_bounds(4) = -100.0;
    settings.lower_bounds(5) = 0.00;
    settings.upper_bounds = optim::ColVec_t::Zero(optim_param_num);
    settings.upper_bounds(0) = 100.0;
    settings.upper_bounds(1) = 1.0;
    settings.upper_bounds(2) = 100.0;
    settings.upper_bounds(3) = 1.0;

    settings.upper_bounds(4) = 100.0;
    settings.upper_bounds(5) = 1.0;
    CurveCostFunction opt_fn_obj(robot_init_pose,target_pose) ;

    opt_fn_obj.weight = std::vector<float>{0.02*0.02,0.1,1.0, 1.0, 0.2, 0.0001, 0.2, 0.2};
    opt_fn_obj.weight = std::vector<float>{0.1, 0.1, 0.02, 0.00001, 0.001, 0.00001};
    {
        //nan
        opt_fn_obj.weight = std::vector<float>{0.1, 0.1, 0.02, 0.01, 0.002, 0.002};

    }
    bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);
//    bool success = optim::gd(x, opt_fn_obj, nullptr,settings);
//    bool success = optim::lbfgs(x, opt_fn_obj, nullptr,settings);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    optim_param = x;
    std::cout << "solution: x = \n" << x << std::endl;

    transform::Transform2d next_pose = updateCurveDIst(robot_init_pose, x(0), x(1));
    next_pose = updateCurveDIst(next_pose, x(2), x(3));
    next_pose = updateCurveDIst(next_pose, x(4), x(5));
    PLOGD<<   "predict final robot_init_pose  = \n" << next_pose << std::endl;

    if(!std::isnormal(x.norm())){
        PLOGD<<   "optim get nan value" << x << std::endl;

        return false;
    }
    return true;

}
int main(int argc, char** argv){


    plog::RollingFileAppender<plog::CsvFormatter> fileAppender("scan_circle.csv", 10000000, 10); // Create the 1st appender.
    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    sol::state lua;
    std::cout << "=== opening a state ===" << std::endl;

    // open some common libraries
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);

    lua.set_exception_handler(&my_exception_handler);

    if(0){
        test_PathSolver(-2.09607, 0.01467, -0.01799 );


        return 0;

    }



    //==== ros
    ros::init(argc, argv, "scan_matching");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");




    std::string mode = "circle";
    const char* mode_param = "mode";
    const char* MODE_CIRCLE = "circle";
    const char* MODE_SHELF4 = "shelf4";
    const char* MODE_SHELF2 = "shelf2";


    // target in base_frame
    // x, y, yaw, init_radius, track_radius, range_offset_radius
    std::vector<float> detect_target_relative_pose{0.0,0.0,0.0,0.0,0.0,0.0};
    const char* detect_target_relative_pose_param = "detect_target_relative_pose";


    transform::Transform2d detect_target_relative_tf;
    transform::Transform2d detect_target_absolute_tf;
    bool detect_target_absolute_pose_computed = false;
    bool detect_target_stop_update = false;
    float detect_target_stop_update_dist = 0.3;
    float stop_plan_dist = 0.4;
    const char* detect_target_stop_update_dist_param = "detect_target_stop_update_dist";

    const char* stop_plan_dist_param = "stop_plan_dist";
    std::array<float,4> shelf_empty_region{0.0,0.0,0.0,0.0};

    // odom_base_tf_1* detect_target_relative_tf_1 = odom_base_tf_2* detect_target_relative_tf_2
    // detect_target_relative_tf_2 = odom_base_tf_2.inverse()*odom_base_tf_1* detect_target_relative_tf_1

    transform::Transform2d odom_base_tf_1, odom_base_tf_2;


    // shelf len_x, len_y
    std::vector<float> shelf_len_y_x{0.0,0.0};
    std::vector<float> shelf_len_y_x_real{0.0,0.0};

    const char* shelf_len_y_x_param = "shelf_len_y_x";
    const char* shelf_len_y_x_real_param = "shelf_len_y_x_real";

    /*
     1. first detect shelf
     maybe 4 legs or 2 legs
     define shelf_center_pose_in_base, shelf_len_y, shelf_len_x, search_radius

     2. detect
     compute leg pose
     filter and segment
     find empty region doesn't contain points: shelf center, line center
     or big box cover robot ase and leg center

     match with shelf_len_y shelf_len_x

     find center line, equally divide L1_L2 L3_L4


     L1----------------------------L2


                   <> target_origin


     L3----------------------------L4


                   ^ x
                   |
                   |
          y<-------robot
     * */

    /*

     3. final control pose

     define
     distance: robot to L1_L2_center
     angle: angle between robot_x and shelf_origin_to_L1_L2_center


     4. control
     robot move to target pose without rotation
     check tolerance
     if not in tolerance, move back and move forward again
     if in tolerance, rotate to target angle


     L1-------------O---------------L2


                   ^ x
                   |
                   |
          y<-------robot



     L3----------------------------L4

     * */


    float front_radius = 0.03;
    const char* front_radius_param = "front_radius";
    int front_min_num = 5;
    const char* front_min_num_param = "front_min_num";



    std::vector<geometry::DetectTarget > shelf_leg_base_pose;
    std::vector<std::vector<geometry::DetectTarget>> shelf_leg_front_pose;
    std::vector<int> shelf_leg_important_index{0,1,2,3};
    std::vector< std::array<int,10>> shelf_leg_match_result;
    float pattern_match_radius = 0.05;
    const char* pattern_match_radius_param = "pattern_match_radius";
    int shelf_leg_min_matched_num = 4;
    float shelf_max_fit_error = 0.04;
    std::vector<float> shelf_rect_diff{0.05,0.05, 0.05}; // edge1_diff, edge2_diff, angle_diff
    const char* shelf_rect_diff_param = "shelf_rect_diff";



//    std::vector<geometry::DetectTarget > shelf_leg_laser_pose;


    float sleep_time = 50.0;
    const char* sleep_time_param = "sleep_time";

    float tf_wait_time = 0.08;
    const char* tf_wait_time_param = "tf_wait_time";

    float range_max = 30.0;
    const char* range_max_param = "range_max";
    float range_min = 1.0;
    const char* range_min_param = "range_min";

    float filer_angle_min = - 1.57;
    float filer_angle_max = 1.57;
    const char* filer_angle_min_param = "filer_angle_min";
    const char* filer_angle_max_param = "filer_angle_max";


    float scan_point_jump = 0.06;
    float scan_noise_angle = 0.06;
    const char* scan_point_jump_param = "scan_point_jump";
    const char* scan_noise_angle_param = "scan_noise_angle";

    float box_filter_min_x = 0.1;
    float box_filter_max_x = 1.2;

    float box_filter_min_y = -0.4;
    float box_filter_max_y = 0.4;

    const char* box_filter_min_x_param = "box_filter_min_x";
    const char* box_filter_max_x_param = "box_filter_max_x";
    const char* box_filter_min_y_param = "box_filter_min_y";
    const char* box_filter_max_y_param = "box_filter_max_y";


    float circle_radius = 0.1;
    float circle_edge_radius = 0.08;
    int circle_min_num_in_radius = 10;
    float circle_edge_range_offset = 0.03;
    int circle_min_point_num = 10;

    const char* circle_radius_param = "circle_radius";
    const char* circle_edge_radius_param = "circle_edge_radius";
    const char* circle_min_num_in_radius_param = "circle_min_num_in_radius";
    const char* circle_edge_range_offset_param = "circle_edge_range_offset";
    const char* circle_min_point_num_param = "circle_min_point_num";

    float max_fit_error = 0.01;
    const char* max_fit_error_param = "max_fit_error";

    float split_dist_x = 0.08;
    float split_dist_y = 0.08;
    float split_dist_r = 0.05;
    float shadow_angle = M_PI_2 - 0.05;


    int min_point_num_in_seg = 10;
    const char* split_dist_x_param = "split_dist_x";
    const char* split_dist_y_param = "split_dist_y";
    const char* split_dist_r_param = "split_dist_r";
    const char* shadow_angle_param = "shadow_angle";

    const char* min_point_num_in_seg_param = "min_point_num_in_seg";


    bool enable_control = false;
    float control_target_angle = 0.0;
    float control_target_distance = 0.5;
    const char* control_target_angle_param = "control_target_angle";
    const char* control_target_distance_param = "control_target_distance";
    const char* enable_control_param = "enable_control";

    float control_interpolate_dist = 0.05;
    const char* control_interpolate_dist_param = "control_interpolate_dist";

    // if angle_diff is bigger than rotate_angle_up_bound
    // robot should rotate to minimize angle_diff first
    // then move forward
    float rotate_angle_up_bound = 0.3;

    float control_dist_tolerance = 0.01;
    float control_angle_tolerance = 0.01;

    float control_forward_vel_tolerance = 0.01;
    float control_rotate_vel_tolerance = 0.01;

    const char* control_dist_tolerance_param = "control_dist_tolerance";
    const char* control_angle_tolerance_param = "control_angle_tolerance";
    const char* control_forward_vel_tolerance_param = "control_forward_vel_tolerance";
    const char* control_rotate_vel_tolerance_param = "control_rotate_vel_tolerance";


    float rotate_pid_control_kp = 0.1;
    float rotate_pid_control_ki = 0.0;
    float rotate_pid_control_kd = 0.0;
    float rotate_pid_control_scale = 5.0;

    const char* rotate_pid_control_kp_param = "rotate_pid_control_kp";
    const char* rotate_pid_control_ki_param = "rotate_pid_control_ki";
    const char* rotate_pid_control_kd_param = "rotate_pid_control_kd";
    const char* rotate_pid_control_scale_param = "rotate_pid_control_scale";

    float forward_pid_control_kp = 0.1;
    float forward_pid_control_ki = 0.0;
    float forward_pid_control_kd = 0.0;
    float forward_pid_control_scale = 0.0;

    const char* forward_pid_control_kp_param = "forward_pid_control_kp";
    const char* forward_pid_control_ki_param = "forward_pid_control_ki";
    const char* forward_pid_control_kd_param = "forward_pid_control_kd";
    const char* forward_pid_control_scale_param = "forward_pid_control_scale";


    float static_rotate_vel = 0.1;
    float static_forward_vel = 0.1;
    const char* static_rotate_vel_param = "static_rotate_vel";
    const char* static_forward_vel_param = "static_forward_vel";


    float final_control_dist = 0.05;
    const char* final_control_dist_param = "final_control_dist";

    float final_control_angle = 0.05;
    const char* final_control_angle_param = "final_control_angle";



    // control command
    const char* status_param = "status";
    const char* run_param = "run";
    int run_command = 0;
    int run_command_last = 0;
    int start_run = 0;
    int control_finished_cnt = 0;

    //tf
    std::string base_frame = "base_link";
    std::string odom_frame = "odom";
    std::string laser_frame = "base_laser";
    std::string map_frame = "map";
    tf::TransformListener tl_;
    transform::Transform2d tf_base_laser;
    transform::Transform2d tf_base_laser_inv;

    transform::Transform2d tf_odom_base;
    tf::StampedTransform transform;
    bool tf_get_base_laser = false;
    bool tf_get_odom_base = false;

    // laser points

    std::vector<geometry::Point> filter_points_laser(100);
    std::vector<geometry::Point> filter_points_base(100);
    std::vector<std::vector<geometry::Point>> circle_filter_points_segments;
    std::vector<std::vector<std::vector<geometry::Point>>> shelf_filter_points_segments;


    auto compute_leg_pose = [&]{

        if(std::strcmp(mode.c_str(), MODE_SHELF4) == 0){

#if 0
            float leg1[2] = {0.5*shelf_len_y_x[1],0.5*shelf_len_y_x[0]};
            float leg2[2] = {0.5*shelf_len_y_x[1],-0.5*shelf_len_y_x[0]};
            float leg3[2] = {-0.5*shelf_len_y_x[1],0.5*shelf_len_y_x[0]};
            float leg4[2] = {-0.5*shelf_len_y_x[1],-0.5*shelf_len_y_x[0]};

#endif
            float leg1[2] = {0.0, 0.5*shelf_len_y_x[0]};
            float leg2[2] = {0.0, -0.5*shelf_len_y_x[0]};
            float leg3[2] = {-shelf_len_y_x[1],0.5*shelf_len_y_x[0]};
            float leg4[2] = {-shelf_len_y_x[1],-0.5*shelf_len_y_x[0]};


            shelf_leg_base_pose.resize(4);

            detect_target_relative_tf.mul(leg1, 1, shelf_leg_base_pose[0].pose_in_base);
            detect_target_relative_tf.mul(leg2, 1, shelf_leg_base_pose[1].pose_in_base);
            detect_target_relative_tf.mul(leg3, 1, shelf_leg_base_pose[2].pose_in_base);
            detect_target_relative_tf.mul(leg4, 1, shelf_leg_base_pose[3].pose_in_base);

//            shelf_empty_region
            shelf_filter_points_segments.resize(4);
            shelf_leg_front_pose.resize(4);
            shelf_leg_min_matched_num = 4;
        }else if(std::strcmp(mode.c_str(), MODE_SHELF2) == 0){
            float leg1[2] = {0.0,0.5*shelf_len_y_x[0]};
            float leg2[2] = {0.0,-0.5*shelf_len_y_x[0]};

            shelf_leg_base_pose.resize(2);

            detect_target_relative_tf.mul(leg1, 1, shelf_leg_base_pose[0].pose_in_base);
            detect_target_relative_tf.mul(leg2, 1, shelf_leg_base_pose[1].pose_in_base);
            shelf_filter_points_segments.resize(2);
            shelf_leg_front_pose.resize(2);
            shelf_leg_min_matched_num= 2;
        }
    };

    auto load_params = [&]{
        nh_private.getParam(mode_param, mode);

        nh_private.getParam(detect_target_relative_pose_param, detect_target_relative_pose);

        nh_private.getParam(shelf_len_y_x_param, shelf_len_y_x);
        shelf_len_y_x_real = shelf_len_y_x;

        if(detect_target_relative_pose.size() != 6 ){
            std::cerr << "detect_target_relative_pose size error"<< std::endl;
            return -1;
        }


        std::cout << "load detect_target_relative_pose : " << detect_target_relative_pose[0] << ", " << detect_target_relative_pose[1] << ", " << detect_target_relative_pose[2] << ", " << detect_target_relative_pose[3] << std::endl;
        if(std::abs(detect_target_relative_pose[0]) < 0.01){
            std::cerr << "detect_target_relative_pose value error"<< std::endl;

            return -1;
        }
        detect_target_relative_tf.set(detect_target_relative_pose[0],detect_target_relative_pose[1],detect_target_relative_pose[2]);



        if(std::strcmp(mode.c_str(), MODE_SHELF4) == 0){

            if(shelf_len_y_x.size() != 2 ){
                std::cerr << "shelf_len_y_x size error"<< std::endl;

                return -1;
            }

            if(shelf_len_y_x[0] < 0.1 || shelf_len_y_x[1] < 0.1){
                std::cerr << "shelf_len_y_x value error"<< std::endl;
                return -1;
            }

            // compute four legs
            compute_leg_pose();

        }

        if(std::strcmp(mode.c_str(), MODE_SHELF2) == 0){

            if(shelf_len_y_x.size() <= 1 ){
                std::cerr << "shelf_len_y_x size error"<< std::endl;

                return -1;
            }

            if(shelf_len_y_x[0] < 0.1 ){
                std::cerr << "shelf_len_y_x value error"<< std::endl;
                return -1;
            }

            // compute two legs
            compute_leg_pose();
        }



        nh_private.getParam(front_radius_param, front_radius);

        nh_private.getParam(front_min_num_param, front_min_num);


        nh_private.getParam(pattern_match_radius_param, pattern_match_radius);

        nh_private.getParam(shelf_rect_diff_param, shelf_rect_diff);



        nh_private.getParam(detect_target_stop_update_dist_param, detect_target_stop_update_dist);

        nh_private.getParam(stop_plan_dist_param, stop_plan_dist);

        nh_private.getParam(max_fit_error_param, max_fit_error);
        nh_private.getParam(sleep_time_param, sleep_time);
        nh_private.getParam(tf_wait_time_param, tf_wait_time);
        nh_private.getParam(range_max_param, range_max);
        nh_private.getParam(range_min_param, range_min);
        nh_private.getParam(scan_point_jump_param, scan_point_jump);
        nh_private.getParam(scan_noise_angle_param, scan_noise_angle);


        nh_private.getParam(shadow_angle_param, shadow_angle);

        nh_private.getParam(filer_angle_min_param, filer_angle_min);
        nh_private.getParam(filer_angle_max_param, filer_angle_max);

        nh_private.getParam(box_filter_min_x_param, box_filter_min_x);
        nh_private.getParam(box_filter_max_x_param, box_filter_max_x);
        nh_private.getParam(box_filter_min_y_param, box_filter_min_y);
        nh_private.getParam(box_filter_max_y_param, box_filter_max_y);

        nh_private.getParam(circle_radius_param, circle_radius);
        nh_private.getParam(circle_edge_radius_param, circle_edge_radius);
        nh_private.getParam(circle_min_num_in_radius_param, circle_min_num_in_radius);

        nh_private.getParam(circle_edge_range_offset_param, circle_edge_range_offset);


        nh_private.getParam(circle_min_point_num_param, circle_min_point_num);


        nh_private.getParam(split_dist_x_param, split_dist_x);
        nh_private.getParam(split_dist_y_param, split_dist_y);
        nh_private.getParam(split_dist_r_param, split_dist_r);

        nh_private.getParam(min_point_num_in_seg_param, min_point_num_in_seg);

        nh_private.getParam(control_target_angle_param, control_target_angle);
        nh_private.getParam(control_target_distance_param, control_target_distance);
        nh_private.getParam(enable_control_param, enable_control);

        nh_private.getParam(control_interpolate_dist_param, control_interpolate_dist);


        nh_private.getParam(control_dist_tolerance_param, control_dist_tolerance);
        nh_private.getParam(control_angle_tolerance_param, control_angle_tolerance);
        nh_private.getParam(control_forward_vel_tolerance_param, control_forward_vel_tolerance);
        nh_private.getParam(control_rotate_vel_tolerance_param,control_rotate_vel_tolerance);


        nh_private.getParam(static_rotate_vel_param, static_rotate_vel);
        nh_private.getParam(static_forward_vel_param, static_forward_vel);



        nh_private.getParam(rotate_pid_control_kp_param, rotate_pid_control_kp);
        nh_private.getParam(rotate_pid_control_ki_param, rotate_pid_control_ki);
        nh_private.getParam(rotate_pid_control_kd_param, rotate_pid_control_kd);
        nh_private.getParam(forward_pid_control_kp_param, forward_pid_control_kp);
        nh_private.getParam(forward_pid_control_ki_param, forward_pid_control_ki);
        nh_private.getParam(forward_pid_control_kd_param, forward_pid_control_kd);


        nh_private.getParam(final_control_dist_param, final_control_dist);
        nh_private.getParam(final_control_angle_param, final_control_angle);


        return 1;
    };

    start_run = load_params();

    common::Suspend suspend;
    sensor::ScanToPoints scan_handler;

    scan_handler.scan_max_jump = scan_point_jump;
    scan_handler.scan_noise_angle = scan_noise_angle;

    bool scan_get_data = false;
    ros::Time scan_time;
    auto laser_cb = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        laser_frame.assign(msg->header.frame_id);
        scan_time = msg->header.stamp;
        scan_get_data = true;
        scan_handler.getLocalPoints(msg->ranges, msg->angle_min, msg->angle_increment, range_min, range_max,filer_angle_min,filer_angle_max);
        scan_get_data = scan_handler.range_valid_num > 10;
    };
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laser_cb);


    nav_msgs::Odometry robot_odom;
    bool odom_get_data = false;
    auto odom_cb = [&](const  nav_msgs::OdometryConstPtr &msg){
        robot_odom = *msg;
        odom_get_data = true;
    };
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odom_cb);



    const char* cmd_vel_topic = "/cmd_vel";
    const char* cloud_target_topic = "detect_target";
    const char* debug_info_topic = "debug_info";
    const char* cloud_filtered_topic = "cloud_filtered";
    const char* detect_results_topic = "detect_results";

    ros::Publisher cmd_vel_pub =  nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

    geometry_msgs::Twist cmd_vel_msg;

    ros::Publisher cloud_target_pub = nh_private.advertise<visualization_msgs::MarkerArray>(cloud_target_topic,1);

    ros::Publisher debug_info_pub = nh_private.advertise<geometry_msgs::Point>(debug_info_topic,1);
    geometry_msgs::Point debug_info_msg ;

    ros::Publisher cloud_filtered_pub = nh_private.advertise<sensor_msgs::PointCloud2>(cloud_filtered_topic, 1);
    sensor_msgs::PointCloud2 cloud_filtered_msg;

    ros::Publisher detect_results_pub = nh_private.advertise<geometry_msgs::PoseArray>(detect_results_topic, 1);
    geometry_msgs::PoseArray detect_results_msg;
    geometry_msgs::Pose target_pose;


    detect_results_msg.header.frame_id = "base_link";

    cloud_filtered_msg.header.frame_id = "map";

    sensor::createPointCloud2(cloud_filtered_msg, {"x", "y", "z"});

    std::vector<geometry::Point> cloud_filtered_points;


    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker  marker_msg;
    visualization_msgs::Marker  marker_arrow_msg;
    std::vector<visualization_msgs::Marker> marker_arrow_msg_array;

    marker_msg.header.frame_id = "base_link";//fixed_frame;
    marker_msg.type = visualization_msgs::Marker::CYLINDER;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 2*circle_radius;
    marker_msg.scale.y = 2*circle_radius;
    marker_msg.scale.z = 2*circle_radius;
    marker_msg.header.stamp = ros::Time();
    marker_msg.color.a = 0.1; // Don't forget to set the alpha!
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;

    marker_arrow_msg.header.frame_id = "base_link";//fixed_frame;
    marker_arrow_msg.type = visualization_msgs::Marker::ARROW;
    marker_arrow_msg.action = visualization_msgs::Marker::ADD;
    marker_arrow_msg.pose.orientation.w = 1.0;
    marker_arrow_msg.scale.x = 0.1;
    marker_arrow_msg.scale.y = 0.005;
    marker_arrow_msg.scale.z = 0.005;
    marker_arrow_msg.header.stamp = ros::Time();
    marker_arrow_msg.color.a = 0.7; // Don't forget to set the alpha!
    marker_arrow_msg.color.r = 0.0;
    marker_arrow_msg.color.g = 1.0;
    marker_arrow_msg.color.b = 0.0;

    marker_array_msg.markers.resize(1,marker_msg);

    std::vector<float> points_in_base(500);

    int filter_point_num = 0;


    float pid_dt = 0.001*sleep_time;


    PID rotate_pid(pid_dt, 0.1, -0.1, rotate_pid_control_kp, rotate_pid_control_kd, rotate_pid_control_ki);
    common::Time rotate_pid_t1 = common::FromUnixNow();

    PID forward_pid(pid_dt, 0.1, -0.1, forward_pid_control_kp, forward_pid_control_kd, forward_pid_control_ki);



    bool first_detect_ok = false;
    bool curve_path_computed = false;
    std::vector<transform::Transform2d> control_path_pose_array;

    auto pending_task = [&]{
        run_command_last = run_command;
        suspend.sleep(1000.0);
    };
    auto restart_task = [&]{
        run_command_last = run_command;
        first_detect_ok = false;
        curve_path_computed = false;
        detect_target_absolute_pose_computed = false;
        detect_target_stop_update = false;
        start_run = load_params();
        control_finished_cnt = 0;
        rotate_pid.reset();
        forward_pid.reset();
        control_path_pose_array.clear();

    };


    control::PathPlanner path_planner;

    transform::MovementCheck movement_check;
    movement_check.move_translation_epsilon = 0.1;
    movement_check.move_rotation_epsilon = 0.05;
    nh_private.setParam(status_param,0);


    while (ros::ok()){
        nh_private.getParam(run_param,run_command);

        if(run_command == 0){
            pending_task();
            continue;
        }
        if(run_command && !run_command_last){
            restart_task();
        }

        if(start_run == -1 || start_run == 0){
            pending_task();
            continue;
        }


        ros::spinOnce();

        if(scan_get_data){

            // transform points to base frame
            // apply box filter
            //

            if (!tf_get_base_laser && scan_get_data) {
                try {
                    tl_.lookupTransform(base_frame, laser_frame, ros::Time(0), transform);
                    tf_get_base_laser = true;
                    tf_base_laser.set(transform.getOrigin().x(), transform.getOrigin().y(),
                                      tf::getYaw(transform.getRotation()));

                    tf_base_laser_inv = tf_base_laser.inverse();
                } catch (tf::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                    continue;
                }
            }


            tf_get_odom_base = false;
            // lookup odom base_link
            try {
                tl_.waitForTransform(odom_frame, base_frame, scan_time, ros::Duration(tf_wait_time));

                tl_.lookupTransform(odom_frame, base_frame, scan_time, transform);
//                tf_get_odom_base = true;
                tf_odom_base.set(transform.getOrigin().x(), transform.getOrigin().y(),
                                 tf::getYaw(transform.getRotation()));
                tf_get_odom_base = true;

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }
            scan_get_data = false;



            PLOGD << "tf_base_laser: " <<  tf_base_laser << std::endl;

            tf_base_laser.mul(scan_handler.local_xy_points, scan_handler.range_valid_num, points_in_base);

            PLOGD << "range_valid_num: " <<  scan_handler.range_valid_num << std::endl;
#if 0
            for(int i = 0; i < scan_handler.range_valid_num;i++){
                std::cout << "== "<< i << ", " << scan_handler.local_xy_points[i+i] << ", " << scan_handler.local_xy_points[i+i+1]  <<"\n";

            }
#endif


            cloud_filtered_points.clear();

            if(std::strcmp(mode.c_str(), MODE_CIRCLE) == 0){

//                LaserScanBoxFilter(scan_handler.local_xy_points, points_in_base, scan_handler.range_valid_num, filter_points_laser,filter_points_base, filter_point_num,box_filter_min_x,box_filter_max_x,box_filter_min_y,box_filter_max_y );
//                detect_target_absolute_pose_computed? detect_target_relative_pose[4] : detect_target_relative_pose[3]
                float search_radius =
                        (!first_detect_ok)?
                        detect_target_relative_pose[3]
                        +
                        detect_target_relative_pose[5]*sqrt(detect_target_relative_pose[0]*detect_target_relative_pose[0] + detect_target_relative_pose[1]*detect_target_relative_pose[1])
                                          : detect_target_relative_pose[4];


                PLOGD << "first_detect_ok : " << first_detect_ok <<std::endl;

                PLOGD << "check search_radius : " << search_radius << std::endl;


                perception::LaserScanRadiusFilter(scan_handler.local_xy_points, points_in_base, scan_handler.range_valid_num, filter_points_laser,filter_points_base, filter_point_num, detect_target_relative_pose[0],detect_target_relative_pose[1],

                                                  search_radius);


                PLOGD << "after LaserScanRadiusFilter filter_point_num: " << filter_point_num << std::endl;
#if 0
                for(int i = 0; i < filter_point_num;i++){
                std::cout << "** "<< i << ", " << filter_points_laser[i].x << ", " << filter_points_laser[i].y  <<"\n";

            }
#endif

                perception::LaserScanSegment(filter_points_laser,circle_filter_points_segments, filter_point_num,split_dist_x, split_dist_y, split_dist_r, min_point_num_in_seg, shadow_angle);
                PLOGD << "LaserScanSegment circle_filter_points_segments.size() " << circle_filter_points_segments.size()<< std::endl;

                PLOGD << "LaserScanSegment cloud_filtered_points.size() " << cloud_filtered_points.size()<< std::endl;



                std::vector<geometry::DetectTarget> find_result(circle_filter_points_segments.size());
                PLOGD << "FindCircle circle_filter_points_segments.size() " << circle_filter_points_segments.size()<< std::endl;
                PLOGD << "FindCircle check max_fit_error " << max_fit_error << std::endl;

                marker_array_msg.markers[0].header.stamp = scan_time;
//                marker_array_msg.markers[0].header.frame_id.assign(laser_frame);
                marker_array_msg.markers[0].pose.position.x =detect_target_relative_tf.x();
                marker_array_msg.markers[0].pose.position.y =detect_target_relative_tf.y();
                marker_array_msg.markers[0].id = 1;
                marker_array_msg.markers[0].scale.x = 2*search_radius;
                marker_array_msg.markers[0].scale.y = 2*search_radius;
                marker_array_msg.markers[0].scale.z = 2*search_radius;
                marker_array_msg.markers[0].color.r = 0.0;
                marker_array_msg.markers[0].color.g = 1.0;
                marker_array_msg.markers[0].color.b = 0.0;


                for(int i = 0 ; i <circle_filter_points_segments.size(); i++ ){
                    PLOGD << "FindCircle"<< std::endl;

                   perception::FindCircle(circle_filter_points_segments[i], circle_radius,circle_edge_radius,circle_edge_range_offset, circle_min_point_num ,find_result[i].pose_in_laser[0],find_result[i].pose_in_laser[1],find_result[i].match_error);
//                   perception::FindCirclePcl(circle_filter_points_segments[i], circle_radius,circle_edge_radius,circle_edge_range_offset, circle_min_point_num ,find_result[i].pose_in_laser[0],find_result[i].pose_in_laser[1],find_result[i].match_error);

                    tf_base_laser.mul(find_result[i].pose_in_laser, 1, find_result[i].pose_in_base);
                    PLOGD << "FindCircle done, match_error " << find_result[i].match_error << std::endl;
                    PLOGD << "FindCircle done, center_in_laser " << find_result[i].pose_in_laser[0] << ", " << find_result[i].pose_in_laser[1] << std::endl;
                    PLOGD << "FindCircle done, center_in_laser " << find_result[i].pose_in_base[0] << ", " << find_result[i].pose_in_base[1] << std::endl;

                    if(find_result[i].match_error < max_fit_error){
                        cloud_filtered_points.insert(cloud_filtered_points.end(), circle_filter_points_segments[i].begin(),circle_filter_points_segments[i].end());

                    }

                }

                if(!cloud_filtered_points.empty()){

                    PLOGD << "create pointcloud"<< std::endl;
                    sensor::LaserScanToPointCloud2(cloud_filtered_points,cloud_filtered_points.size(),cloud_filtered_msg);
                    cloud_filtered_msg.header.stamp = scan_time;
                    cloud_filtered_msg.header.frame_id.assign(laser_frame);

                    cloud_filtered_pub.publish(cloud_filtered_msg);
                    PLOGD << "create pointcloud done"<< std::endl;

                }







                int best_result_id = 0;
                if(find_result.empty()){
                    best_result_id = -1;
                }else{
                    std::sort(find_result.begin(),find_result.end(),[](auto& v1,auto& v2){
                        return (std::abs(v1.pose_in_base[0])+std::abs(v1.pose_in_base[1])) < ( std::abs(v2.pose_in_base[0]) + std::abs(v2.pose_in_base[1]));

                    });
                    std::sort(find_result.begin(),find_result.end(),[](auto& v1,auto& v2){
                        return  v1.match_error < v2.match_error;
                    });

                    while(find_result[best_result_id].match_error > max_fit_error){

                        best_result_id++;
                        if(best_result_id == find_result.size()){
                            best_result_id = -1;
                            break;
                        }
                    }
                }


                if(best_result_id == -1){

                    if(!detect_target_absolute_pose_computed){
                        PLOGD << "find circle fail " << std::endl;
                        cloud_target_pub.publish(marker_array_msg);

                        continue;
                    }else{
                        if(tf_get_odom_base){


                            detect_target_relative_tf = tf_odom_base.inverse()*detect_target_absolute_tf;

                        }

                    }
                }else{

                    PLOGD << "find best_result_id " << best_result_id <<std::endl;


                    geometry::DetectTarget best_result = find_result[best_result_id];

                    PLOGD << "find best_result.pose_in_laser  " << best_result.pose_in_laser[0] << ", " << best_result.pose_in_laser[1]  <<std::endl;


                    marker_array_msg.markers.resize(2);

                    marker_array_msg.markers[1] = marker_array_msg.markers[0];
                    marker_array_msg.markers[1].pose.position.x = best_result.pose_in_base[0];
                    marker_array_msg.markers[1].pose.position.y = best_result.pose_in_base[1];


                    marker_array_msg.markers[1].scale.x = 2*circle_radius;
                    marker_array_msg.markers[1].scale.y = 2*circle_radius;
                    marker_array_msg.markers[1].scale.z = 2*circle_radius;
                    marker_array_msg.markers[1].id = 2;
                    marker_array_msg.markers[1].color.r = 1.0;
                    marker_array_msg.markers[1].color.g = 0.0;
                    marker_array_msg.markers[1].color.b = 0.0;

                    cloud_target_pub.publish(marker_array_msg);


                    detect_target_relative_tf.set(best_result.pose_in_base[0] , best_result.pose_in_base[1], std::atan2(best_result.pose_in_base[1],best_result.pose_in_base[0]));
                    if(tf_get_odom_base){

                        detect_target_absolute_pose_computed = true;

                        detect_target_absolute_tf = tf_odom_base * detect_target_relative_tf;

                    }


                }

                if(!first_detect_ok){
                    nh_private.setParam(status_param,1);

                }

                first_detect_ok = true;

                PLOGD << "control_target_angle: " << control_target_angle << std::endl;
                PLOGD << "control_target_distance: " << control_target_distance << std::endl;
                PLOGD << "target pose in base ,  " << detect_target_relative_tf.x() << ", " << detect_target_relative_tf.y() << std::endl;

                detect_target_relative_pose[0] = detect_target_relative_tf.x();
                detect_target_relative_pose[1] = detect_target_relative_tf.y();





                float marker_angle = detect_target_relative_tf.yaw();

                marker_angle = std::abs(marker_angle - control_target_angle) < M_PI ? (marker_angle) : ( marker_angle + ((marker_angle- control_target_angle) > 0.0 ? -M_PI*2: M_PI*2) );

                float angle_diff = marker_angle - control_target_angle;

                float marker_distance = std::sqrt(detect_target_relative_tf.x()*detect_target_relative_tf.x() + detect_target_relative_tf.y()*detect_target_relative_tf.y() );
                float distance_diff = marker_distance - control_target_distance;

                PLOGD << "angle_diff: " << angle_diff << std::endl;
                PLOGD << "distance_diff: " << distance_diff << std::endl;

                {
                    if( std::abs(angle_diff) > final_control_angle){
                        cmd_vel_msg.angular.z =  final_control_angle*rotate_pid_control_kp*(angle_diff> 0.0 ? 1.0:-1.0);;

                    }else{
                        double rotate_pid_inc = rotate_pid.calculate(angle_diff, 0.0);
                        PLOGD << "rotate_pid_inc: " << rotate_pid_inc << std::endl;

                        if(std::abs(rotate_pid_inc) < control_rotate_vel_tolerance || std::abs(angle_diff) < control_angle_tolerance){
                            rotate_pid_inc = 0.0;
                        }
                        cmd_vel_msg.angular.z =  rotate_pid_inc;

                    }
                }





                float target_pose_x = distance_diff * std::cos(marker_angle);


//            if(std::abs(angle_diff) < 0.02)
                {

                    if(std::abs(target_pose_x) > final_control_dist){
                        cmd_vel_msg.linear.x =  final_control_dist*forward_pid_control_kp*(target_pose_x> 0.0 ? 1.0:-1.0);
                    }else{
                        double forward_pid_inc = forward_pid.calculate(target_pose_x, 0.0);

                        PLOGD << "forward_pid_inc: " << forward_pid_inc << std::endl;
                        if(std::abs(forward_pid_inc) < control_forward_vel_tolerance || std::abs(target_pose_x) < control_dist_tolerance){
                            forward_pid_inc = 0.0;
                        }
                        cmd_vel_msg.linear.x =  forward_pid_inc;
                    }

                    debug_info_msg.x =target_pose_x;
                    debug_info_msg.y =marker_angle;

                    debug_info_pub.publish(debug_info_msg);

                }


                if(!enable_control){
                    continue;
                }
                PLOGD << "cmd_vel_msg.angular.z : " <<   cmd_vel_msg.angular.z  << std::endl;
                PLOGD << "cmd_vel_msg.linear.x : " <<  cmd_vel_msg.linear.x << std::endl;
                cmd_vel_pub.publish(cmd_vel_msg);

                if(std::abs(target_pose_x)<control_dist_tolerance && std::abs(angle_diff) < control_angle_tolerance){
                    PLOGD << "control finished test : " <<  control_finished_cnt << std::endl;

                    control_finished_cnt++;
                    if(control_finished_cnt > 10){
                        start_run = 0;
                    }
                }
            }else if(std::strcmp(mode.c_str(), MODE_SHELF4) == 0){

//                compute_leg_pose();
                // filter and segment
                // each leg has a clusters of points : std::vector<std::vector<Point>>

                marker_array_msg.markers.resize(shelf_leg_base_pose.size(),marker_msg);
                shelf_leg_front_pose.resize(shelf_leg_base_pose.size());

                detect_results_msg.poses.clear();
                shelf_filter_points_segments.resize(shelf_leg_base_pose.size());

                for(int i =0 ; i < shelf_leg_base_pose.size(); i++){


                    PLOGD << "check leg: " << i << ", " << shelf_leg_base_pose[i].pose_in_base[0] << ", " << shelf_leg_base_pose[i].pose_in_base[1] << std::endl;

                    tf_base_laser_inv.mul(shelf_leg_base_pose[i].pose_in_base, 1, shelf_leg_base_pose[i].pose_in_laser);

                    float search_radius =
                            (!first_detect_ok)?
                            detect_target_relative_pose[3]
                            +
                            detect_target_relative_pose[5]*sqrt(shelf_leg_base_pose[i].pose_in_base[0]*shelf_leg_base_pose[i].pose_in_base[0] + shelf_leg_base_pose[i].pose_in_base[1]*shelf_leg_base_pose[i].pose_in_base[1])
: detect_target_relative_pose[4];


                    PLOGD << "first_detect_ok : " << first_detect_ok <<std::endl;

                    PLOGD << "check search_radius : " << search_radius << std::endl;


                    perception::LaserScanRadiusFilter(scan_handler.local_xy_points, points_in_base, scan_handler.range_valid_num, filter_points_laser,filter_points_base, filter_point_num,
                                          shelf_leg_base_pose[i].pose_in_base[0],shelf_leg_base_pose[i].pose_in_base[1],search_radius
);
                    PLOGD << "check filter_points_laser size : " << filter_points_laser.size() << std::endl;
                    PLOGD << "check filter_point_num : " << filter_point_num << std::endl;

                    perception::LaserScanSegment(filter_points_laser,shelf_filter_points_segments[i], filter_point_num,split_dist_x, split_dist_y, split_dist_r, min_point_num_in_seg);



                    shelf_leg_front_pose[i].resize(shelf_filter_points_segments[i].size());
                    for(int j = 0 ; j < shelf_filter_points_segments[i].size();j++){

                        perception::FindFrontEdge(shelf_filter_points_segments[i][j],3, front_radius, front_min_num ,shelf_leg_front_pose[i][j].pose_in_laser[0], shelf_leg_front_pose[i][j].pose_in_laser[1],shelf_leg_front_pose[i][j].match_error );

                        if(shelf_leg_front_pose[i][j].match_error < 0.05){
                            tf_base_laser.mul(shelf_leg_front_pose[i][j].pose_in_laser, 1, shelf_leg_front_pose[i][j].pose_in_base);

                            PLOGD << "check FindFrontEdge pose_in_laser: " << shelf_leg_front_pose[i][j].pose_in_laser[0] << ", " << shelf_leg_front_pose[i][j].pose_in_laser[1]  << std::endl;
                            PLOGD << "check FindFrontEdge pose_in_base : " << shelf_leg_front_pose[i][j].pose_in_base[0] << ", " << shelf_leg_front_pose[i][j].pose_in_base[1]  << std::endl;
                            target_pose.position.x = shelf_leg_front_pose[i][j].pose_in_laser[0];
                            target_pose.position.y = shelf_leg_front_pose[i][j].pose_in_laser[1];

                            math::yaw_to_quaternion(std::atan2(shelf_leg_front_pose[i][j].pose_in_laser[1],shelf_leg_front_pose[i][j].pose_in_laser[0]),
                                                    target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);

                            detect_results_msg.poses.emplace_back(target_pose);

                        }

                        cloud_filtered_points.insert(cloud_filtered_points.end(),shelf_filter_points_segments[i][j].begin(),shelf_filter_points_segments[i][j].end());

                    }


//                    perception::FindMatchPattern()

                    marker_array_msg.markers[i].header.stamp = scan_time;
//                    marker_array_msg.markers[i].scale.x = 2.0*(detect_target_absolute_pose_computed? detect_target_relative_pose[4] : detect_target_relative_pose[3] );
//                    marker_array_msg.markers[i].scale.y = marker_array_msg.markers[i].scale.x;

//                    marker_array_msg.markers[i].header.frame_id.assign(laser_frame);
                    marker_array_msg.markers[i].scale.x = 2*search_radius;
                    marker_array_msg.markers[i].scale.y = 2*search_radius;
                    marker_array_msg.markers[i].scale.z = 2*search_radius;
                    marker_array_msg.markers[i].pose.position.x = shelf_leg_base_pose[i].pose_in_base[0];
                    marker_array_msg.markers[i].pose.position.y = shelf_leg_base_pose[i].pose_in_base[1];
                    marker_array_msg.markers[i].id = i;

                }



                /*
                 1. cluster segment result save in  shelf_filter_points_segments
                 shelf_filter_points_segments : [ L1:[c1:[p1,p2,p3],c2,c3] ,L2:[c1,c2,c3],L3:[],L4:[]  ]


                 2. for each cluster
                 compute front end pose in laser frame
                 check L1 L2 with leg distance
                 only at first time, check L3 L4 with leg distance, if L3 L4 exist

                 compute L1 L2 direction center

                 */


                perception::FindMatchPattern(shelf_leg_base_pose,shelf_leg_front_pose,pattern_match_radius, shelf_leg_min_matched_num,shelf_leg_match_result );
                PLOGD << "shelf_leg_match_result.size " << shelf_leg_match_result.size() << std::endl;

                int best_result_id = 0;
                if(shelf_leg_match_result.empty()){
                    best_result_id = -1;
                }else{
                    std::sort(shelf_leg_match_result.begin(),shelf_leg_match_result.end(), [](auto& v1, auto& v2){

                        return v1.back() < v2.back();
                    });


                    float best_edge_12 = 1000.0;
                    int best_edge_id = -1;

                    auto& model_leg1 = shelf_leg_base_pose[0];
                    auto& model_leg2 = shelf_leg_base_pose[1];
                    auto& model_leg3 = shelf_leg_base_pose[2];
                    auto& model_leg4 = shelf_leg_base_pose[3];
                    float model_edge_12 = std::sqrt( (model_leg2.pose_in_laser[0] - model_leg1.pose_in_laser[0])*(model_leg2.pose_in_laser[0] - model_leg1.pose_in_laser[0]) +  (model_leg2.pose_in_laser[1] - model_leg1.pose_in_laser[1])*(model_leg2.pose_in_laser[1] - model_leg1.pose_in_laser[1])   );
                    float model_edge_13 = std::sqrt( (model_leg3.pose_in_laser[0] - model_leg1.pose_in_laser[0])*(model_leg3.pose_in_laser[0] - model_leg1.pose_in_laser[0]) +  (model_leg3.pose_in_laser[1] - model_leg1.pose_in_laser[1])*(model_leg3.pose_in_laser[1] - model_leg1.pose_in_laser[1])   );

                    //
                    if(!first_detect_ok){




                        while(best_result_id < shelf_leg_match_result.size()){


                            /*

                             l1     l2


                             l3     l4

                                 |x
                               y-| robot

                             *
                             * */
                            auto& result = shelf_leg_match_result[best_result_id];

                            PLOGD << "check match result: " << result[0] << ", " << result[1] << ", " << result[2] << ", " << result[3] << std::endl;
                            auto& detect_leg1 = shelf_leg_front_pose[0][result[0]];
                            auto& detect_leg2 = shelf_leg_front_pose[1][result[1]];

                            auto& detect_leg3 = shelf_leg_front_pose[2][result[2]];
                            auto& detect_leg4 = shelf_leg_front_pose[3][result[3]];

                            float edge_12 = std::sqrt( (detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );
                            float edge_34 = std::sqrt( (detect_leg4.pose_in_laser[0] - detect_leg3.pose_in_laser[0])*(detect_leg4.pose_in_laser[0] - detect_leg3.pose_in_laser[0]) +  (detect_leg4.pose_in_laser[1] - detect_leg3.pose_in_laser[1])*(detect_leg4.pose_in_laser[1] - detect_leg3.pose_in_laser[1])   );

                            float edge_13 = std::sqrt( (detect_leg3.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg3.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg3.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg3.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );
                            float edge_24 = std::sqrt( (detect_leg4.pose_in_laser[0] - detect_leg2.pose_in_laser[0])*(detect_leg4.pose_in_laser[0] - detect_leg2.pose_in_laser[0]) +  (detect_leg4.pose_in_laser[1] - detect_leg2.pose_in_laser[1])*(detect_leg4.pose_in_laser[1] - detect_leg2.pose_in_laser[1])   );



                            float angle31 = std::atan2(detect_leg1.pose_in_laser[1] - detect_leg3.pose_in_laser[1]  ,detect_leg1.pose_in_laser[0] - detect_leg3.pose_in_laser[0]);
                            float angle42 = std::atan2(detect_leg2.pose_in_laser[1] - detect_leg4.pose_in_laser[1]  ,detect_leg2.pose_in_laser[0] - detect_leg4.pose_in_laser[0]);
                            float angle21 = std::atan2(detect_leg1.pose_in_laser[1] - detect_leg2.pose_in_laser[1]  ,detect_leg1.pose_in_laser[0] - detect_leg2.pose_in_laser[0]);
                            float angle43 = std::atan2(detect_leg3.pose_in_laser[1] - detect_leg4.pose_in_laser[1]  ,detect_leg3.pose_in_laser[0] - detect_leg4.pose_in_laser[0]);

                            float rect_angle1 = angle21 - angle_normalise(angle31, angle21);
                            float rect_angle2 = angle21 - angle_normalise(angle42, angle21);
                            float rect_angle3 = angle43 - angle_normalise(angle31, angle43);
                            float rect_angle4 = angle43 - angle_normalise(angle42, angle43);

                            char msg[500];
                            sprintf(msg,"shelf_len_y_x_real: [%.3f,%.3f], [%.3f,%.3f], angle31:%.3f, angle42:%.3f, angle21:%.3f, angle43:%.3f, rect_angle1:%.3f, rect_angle2:%.3f, rect_angle3:%.3f, rect_angle4:%.3f",
                                    edge_12, edge_13 , edge_34 ,edge_24,angle31,angle42 ,angle21,angle43, rect_angle1, rect_angle2, rect_angle3, rect_angle4);
                            PLOGD << msg << std::endl;

                            bool valid = (std::abs(edge_12 - edge_34) <  shelf_rect_diff[0]) && (edge_12 > shelf_rect_diff[0] )&& (edge_34 > shelf_rect_diff[0])
                                    && (std::abs(edge_13 - edge_24) <  shelf_rect_diff[1] )&& (edge_13 > shelf_rect_diff[1] )&& (edge_24 > shelf_rect_diff[1])
                                    && (std::abs(rect_angle1 - M_PI_2) < shelf_rect_diff[2])
                                    && (std::abs(rect_angle2 - M_PI_2) < shelf_rect_diff[2])
                                    && (std::abs(rect_angle3 - M_PI_2) < shelf_rect_diff[2])
                                    && (std::abs(rect_angle4 - M_PI_2) < shelf_rect_diff[2])
//                                    && std::abs(detect_leg1.pose_in_laser[0]) > std::abs(detect_leg3.pose_in_laser[0])
//                                    && std::abs(detect_leg2.pose_in_laser[0]) > std::abs(detect_leg4.pose_in_laser[0])

                                    ;
                            sprintf(msg,"check valid 1 : %.3f, %.3f, %.3f, %.3f, %.3f, %.3f  ", edge_12 - edge_34, edge_13 - edge_24 , std::abs(rect_angle1 - M_PI_2) ,std::abs(rect_angle2 - M_PI_2),std::abs(rect_angle3 - M_PI_2),std::abs(rect_angle4 - M_PI_2) );
                            PLOGD << msg << std::endl;

                            sprintf(msg,"check shelf_rect_diff  : %.3f, %.3f, %.3f",   shelf_rect_diff[0], shelf_rect_diff[1],shelf_rect_diff[2] );
                            PLOGD << msg << std::endl;


                            sprintf(msg,"check valid 2 : %d, %d, %d, %d, %d, %d, %d, %d  ",
                                    (std::abs(edge_12 - edge_34) <  shelf_rect_diff[0]),
                                    ((edge_12 > shelf_rect_diff[0] )&& (edge_34 > shelf_rect_diff[0])),
                                    (std::abs(edge_13 - edge_24) <  shelf_rect_diff[1] ),
                                    ((edge_13 > shelf_rect_diff[1] )&& (edge_24 > shelf_rect_diff[1])),
                                    (std::abs(rect_angle1 - M_PI_2) < shelf_rect_diff[2])
                                    , (std::abs(rect_angle2 - M_PI_2) < shelf_rect_diff[2])
                                    , (std::abs(rect_angle3 - M_PI_2) < shelf_rect_diff[2])
                                    , (std::abs(rect_angle4 - M_PI_2) < shelf_rect_diff[2])
                            );
                                    PLOGD << msg << std::endl;

                            float dist = std::sqrt( (model_leg2.pose_in_laser[0] - detect_leg2.pose_in_laser[0])*(model_leg2.pose_in_laser[0] - detect_leg2.pose_in_laser[0]) +  (model_leg1.pose_in_laser[1] - model_leg1.pose_in_laser[1])*(model_leg1.pose_in_laser[1] - model_leg1.pose_in_laser[1])   );


                            float cost = std::abs(edge_12 - model_edge_12)
                                    + std::abs(edge_13 - model_edge_13)
                                    + dist;


                            result.back() = 500.0f*(std::abs(edge_12 - edge_34) + std::abs(edge_13 - edge_24));
                            if(valid){

                                if(cost < best_edge_12){

                                    best_edge_12 = cost;
                                    best_edge_id = best_result_id;

                                    PLOGD << "get valid rect, write to file" << std::endl;

                                    shelf_len_y_x_real[0] = edge_12;
                                    shelf_len_y_x_real[1] = edge_34;
                                    shelf_leg_base_pose.resize(2);
                                    shelf_leg_min_matched_num = 2;


                                    nh_private.setParam(shelf_len_y_x_real_param, shelf_len_y_x_real);
                                    std::ofstream t("/tmp/shelf.txt");
                                    t << msg;
                                }
                            }

                            best_result_id++;
                            if(best_result_id == shelf_leg_match_result.size()){
                                best_result_id = -1;
                                break;
                            }
                        }

                        best_result_id = best_edge_id;



                    }else{

                        while(best_result_id < shelf_leg_match_result.size()){

                            auto& result = shelf_leg_match_result[best_result_id];

                            PLOGD << "check match result: " << result[0] << ", " << result[1] << ", " << result[2] << ", " << result[3] << std::endl;
                            auto& detect_leg1 = shelf_leg_front_pose[0][result[0]];
                            auto& detect_leg2 = shelf_leg_front_pose[1][result[1]];


                            float edge_12 = std::sqrt( (detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );



                            float dist = std::sqrt( (model_leg2.pose_in_laser[0] - detect_leg2.pose_in_laser[0])*(model_leg2.pose_in_laser[0] - detect_leg2.pose_in_laser[0]) +  (model_leg1.pose_in_laser[1] - model_leg1.pose_in_laser[1])*(model_leg1.pose_in_laser[1] - model_leg1.pose_in_laser[1])   );


                            float cost = std::abs(edge_12 - model_edge_12) + dist;

                            if(cost < best_edge_12){

                                best_edge_12 = cost;
                                best_edge_id = best_result_id;

                            }

                            best_result_id++;
                            if(best_result_id == shelf_leg_match_result.size()){
                                best_result_id = -1;
                                break;
                            }

                        }
                        best_result_id = best_edge_id;


                    }
                }




                if(detect_target_absolute_pose_computed && detect_target_stop_update){
                    best_result_id = -1;
                }

                if(best_result_id == -1){

                    if(!detect_target_absolute_pose_computed){
                        PLOGD << "find target fail " << std::endl;

                        detect_results_msg.header.stamp = scan_time;
                        detect_results_msg.header.frame_id.assign(laser_frame);

                        detect_results_pub.publish(detect_results_msg);
                        cloud_target_pub.publish(marker_array_msg);

                        continue;
                    }else{
                        if(tf_get_odom_base){
                            detect_target_relative_tf = tf_odom_base.inverse()*detect_target_absolute_tf;

                        }

                    }
                }else{


                    // check matched legs
                    auto& best_result = shelf_leg_match_result[best_result_id];
                    PLOGD << "check match best_result: " << best_result[0] << ", " << best_result[1] << ", " << best_result[2] << ", " << best_result[3] << std::endl;

                    auto& detect_leg1 = shelf_leg_front_pose[0][best_result[0]];
                    auto& detect_leg2 = shelf_leg_front_pose[1][best_result[1]];
                    float edge_12 = std::sqrt( (detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0])*(detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) +  (detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])*(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1])   );
                    shelf_len_y_x_real[0] = edge_12;


                    PLOGD << "check two leg" << std::endl;

                    PLOGD << "detect_leg1: " <<  detect_leg1.pose_in_base[0] << ", " << detect_leg1.pose_in_base[1] << std::endl;
                    PLOGD << "detect_leg2: " <<  detect_leg2.pose_in_base[0] << ", " << detect_leg2.pose_in_base[1] << std::endl;



                    target_pose.position.x = 0.5*(detect_leg1.pose_in_laser[0] + detect_leg2.pose_in_laser[0]);
                    target_pose.position.y = 0.5*(detect_leg1.pose_in_laser[1] + detect_leg2.pose_in_laser[1]);

                    math::yaw_to_quaternion(
 (std::atan2(detect_leg2.pose_in_laser[1] - detect_leg1.pose_in_laser[1]  ,detect_leg2.pose_in_laser[0] - detect_leg1.pose_in_laser[0]) + M_PI_2)
//                    + 0.25*(std::atan2(detect_leg2.pose_in_laser[1] ,detect_leg2.pose_in_laser[0]))
//                      + 0.25*(std::atan2(detect_leg1.pose_in_laser[1] ,detect_leg1.pose_in_laser[0]))

                                            ,
                                            target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);
                    PLOGD << "target_pose: " << target_pose.position.x << ", " << target_pose.position.y << std::endl;



                    detect_results_msg.poses.emplace_back(target_pose);


                    detect_target_relative_tf.set(0.5*(detect_leg1.pose_in_base[0] + detect_leg2.pose_in_base[0]),
                                                  0.5*(detect_leg1.pose_in_base[1] + detect_leg2.pose_in_base[1]),
                                                  std::atan2(detect_leg2.pose_in_base[1] - detect_leg1.pose_in_base[1]  ,detect_leg2.pose_in_base[0] - detect_leg1.pose_in_base[0]) + M_PI_2
                                                  );

                    if(tf_get_odom_base){
                        detect_target_absolute_pose_computed = true;
                        detect_target_absolute_tf = tf_odom_base * detect_target_relative_tf;

                    }
                }

                if(!first_detect_ok){
                    movement_check.reset();
                    nh_private.setParam(status_param,1);

                }

                first_detect_ok = true;



                PLOGD << "control_target_angle: " << control_target_angle << std::endl;
                PLOGD << "control_target_distance: " << control_target_distance << std::endl;


                float leg1[2] = {0.0, 0.5*shelf_len_y_x_real[0]};
                float leg2[2] = {0.0, -0.5*shelf_len_y_x_real[0]};
                float control_target_in_shelf[2] = {-control_target_distance,0.0};
                float control_target_in_base[2] = {0.0,0.0};
                float control_interpolate_target_in_base[2] = {0.0,0.0};

                detect_target_relative_tf.mul(leg1, 1, shelf_leg_base_pose[0].pose_in_base);
                detect_target_relative_tf.mul(leg2, 1, shelf_leg_base_pose[1].pose_in_base);

                detect_target_relative_tf.mul(control_target_in_shelf, 1, control_target_in_base);
                PLOGD << "control_target_in_shelf: " << control_target_in_shelf[0]<<", " << control_target_in_shelf[1] << std::endl;
                PLOGD << "control_target_in_base: " << control_target_in_base[0]<<", " << control_target_in_base[1] << std::endl;

                float marker_distance = std::sqrt(control_target_in_base[0]*control_target_in_base[0] + control_target_in_base[1]*control_target_in_base[1]   );

                float interpolate_distance =  control_target_distance + ( (marker_distance> control_interpolate_dist)?(marker_distance -control_interpolate_dist):(0.0)   );
                control_target_in_shelf[0] = -interpolate_distance;
                detect_target_relative_tf.mul(control_target_in_shelf, 1, control_interpolate_target_in_base);
                PLOGD << "interpolate control_interpolate_target_in_base: " << control_interpolate_target_in_base[0]<<", " << control_interpolate_target_in_base[1] << std::endl;


                auto control_target = detect_target_relative_tf;
                control_target.set(control_target_in_base[0],control_target_in_base[1],detect_target_relative_tf.yaw());
                PLOGD << "start optim" << std::endl;
                transform::Transform2d opt_target_pose;
                transform::Transform2d opt_init_pose =control_target.inverse() ;

                std::cout << "opt_init_pose: = \n" << opt_init_pose << std::endl;
                std::cout << "opt_target_pose: = \n" << opt_target_pose << std::endl;

                bool need_curve_path_compute = !curve_path_computed;

                movement_check.checkMoveTrigger(tf_odom_base);

                if(
                        std::abs(opt_init_pose.x()) > stop_plan_dist
                && movement_check.isMoveTriggered()
                ){
                    need_curve_path_compute = true;
                    movement_check.resetTrigger();

                }

                std::cout << "need_curve_path_compute: = " << need_curve_path_compute << std::endl;


                if(need_curve_path_compute){



                    size_t optim_param_num = 6;
                    Eigen::VectorXd x(optim_param_num);
                    x << 1e-5, 0.1 ,1e-5,0.1,1e-5,0.1;

//                    bool rt = solve_curve_path_plan(opt_init_pose,opt_target_pose, x );
//                    x = test_PathSolver(opt_init_pose.x(),opt_init_pose.y(),opt_init_pose.yaw());
                    bool rt = path_planner.solve(opt_init_pose,opt_target_pose);
                    x = path_planner.solved_params;

                    if(rt){


                        std::cout << "solution: x = \n" << x << std::endl;
                        float curve = 0.0;
                        float dist = 0.0;
                        int N= 10;
                        float dist_inc = x(1)/float(N);
                        transform::Transform2d next_pose;

                        next_pose = updateCurveDIst(opt_init_pose, x(0), x(1));
                        next_pose = updateCurveDIst(next_pose, x(2), x(3));
                        next_pose = updateCurveDIst(next_pose, x(4), x(5));
                        curve_path_computed =(

                                                     ((next_pose.x() - opt_target_pose.x())*(next_pose.x() - opt_target_pose.x()) +
                                                      (next_pose.y() - opt_target_pose.y())*(next_pose.y() - opt_target_pose.y())) < control_dist_tolerance*control_dist_tolerance

                                             )&&(
                                                     std::abs( next_pose.yaw() - opt_target_pose.yaw() ) < control_angle_tolerance
                                             )
                                ;
                        PLOGD<<   "predict final opt_init_pose  = \n" << next_pose << std::endl;

                        PLOGD<<   "check final curve_path_computed  =  " << curve_path_computed << std::endl;

                        if(curve_path_computed){
                            control_path_pose_array.clear();
                            marker_arrow_msg_array.clear();


                            for(size_t i = 0; i < path_planner.path_segments.size(); i++){
                                auto& path = path_planner.path_segments[i];
//                                marker_arrow_msg.color.r = 1.0 - 0.1*(i+1);
//                                marker_arrow_msg.color.g = 1.0 - 0.1*(i+1);
                                marker_arrow_msg.color.b = 1.0 - 0.1*(i+1);
                                for(auto &p : path.path){
                                    next_pose = control_target*p;
                                    marker_arrow_msg.pose.position.x = next_pose.x();
                                    marker_arrow_msg.pose.position.y = next_pose.y();
                                    math::yaw_to_quaternion( next_pose.yaw(),
                                                             marker_arrow_msg.pose.orientation.x,marker_arrow_msg.pose.orientation.y,marker_arrow_msg.pose.orientation.z,marker_arrow_msg.pose.orientation.w);
                                    marker_arrow_msg.id = 100+marker_arrow_msg_array.size();
                                    marker_arrow_msg.header.stamp = scan_time;
                                    marker_arrow_msg_array.push_back(marker_arrow_msg);
                                }
                            }
                        }

                    }

                }


                float local_curve, local_dist;
                bool local_compute_ok = path_planner.computeLocalCurve(opt_init_pose,local_curve,local_dist );


                if(local_compute_ok){
                    std::cout << "local_curve: = " << local_curve << ", local_dist: = " << local_dist << std::endl;


                    float dist_to_final_target = std::sqrt(opt_init_pose.x()*opt_init_pose.x() + opt_init_pose.y()*opt_init_pose.y());


                    PLOGD << "detect_target_stop_update: " << detect_target_stop_update << std::endl;

                    if(dist_to_final_target < detect_target_stop_update_dist){
                        detect_target_stop_update = false;
                    }


                    float local_speed = ( (dist_to_final_target > 0.1)? (0.6) : (dist_to_final_target * 5 + 0.1))*static_forward_vel*( (std::abs(local_curve) < 10) ? ( 1.0 - 0.1*std::abs(local_curve)) :(0.0) ) ;

                    float local_time = std::abs(local_dist)/std::abs(local_speed);

                    float local_angle = local_curve*local_dist;

                    float local_rotate_speed = local_angle/local_time;
                    cmd_vel_msg.angular.z = local_rotate_speed;
                    cmd_vel_msg.linear.x =  local_speed;

                    PLOGD << "cmd_vel_msg.angular.z : " <<   cmd_vel_msg.angular.z  << std::endl;
                    PLOGD << "cmd_vel_msg.linear.x : " <<  cmd_vel_msg.linear.x << std::endl;
                    if(enable_control){
                        cmd_vel_pub.publish(cmd_vel_msg);

                    }

                    if(std::abs(opt_init_pose.x())<control_dist_tolerance
//                && std::abs(opt_init_pose.yaw()) < control_angle_tolerance
                            ){

                        cmd_vel_msg.angular.z = 0;
                        cmd_vel_msg.linear.x =  0;

                        cmd_vel_pub.publish(cmd_vel_msg);
                        PLOGD << "control finished test : " <<  control_finished_cnt << std::endl;

                        control_finished_cnt++;
                        if(control_finished_cnt > 10){

                            if(std::abs(opt_init_pose.yaw()) < control_angle_tolerance
                            && std::abs(opt_init_pose.y())<control_dist_tolerance
                            ){
                                nh_private.setParam(status_param,3);

                            }else{
                                nh_private.setParam(status_param,2);
                            }
                            start_run = 0;
                        }
                    }



                }



                marker_array_msg.markers.insert(marker_array_msg.markers.end(),marker_arrow_msg_array.begin(),marker_arrow_msg_array.end() );
                for(auto&m :marker_array_msg.markers){
                    m.header.stamp = scan_time;
                }
                cloud_target_pub.publish(marker_array_msg);


                sensor::LaserScanToPointCloud2(cloud_filtered_points,cloud_filtered_points.size(),cloud_filtered_msg);
                cloud_filtered_msg.header.stamp = scan_time;
                cloud_filtered_msg.header.frame_id.assign(laser_frame);

                cloud_filtered_pub.publish(cloud_filtered_msg);

                detect_results_msg.header.stamp = scan_time;
                detect_results_msg.header.frame_id.assign(laser_frame);

                detect_results_pub.publish(detect_results_msg);

                if(0){
                    float marker_angle = std::atan2(control_interpolate_target_in_base[1],control_interpolate_target_in_base[0]);
                    float angle_diff = marker_angle - control_target_angle;

                    marker_distance = std::sqrt(control_interpolate_target_in_base[0]*control_interpolate_target_in_base[0] + control_interpolate_target_in_base[1]*control_interpolate_target_in_base[1]   );
                    float distance_diff = marker_distance ;//- control_target_distance;

                    PLOGD << "angle_diff: " << angle_diff << std::endl;
                    PLOGD << "distance_diff: " << distance_diff << std::endl;




                    {
                        if( std::abs(angle_diff) > final_control_angle){
                            cmd_vel_msg.angular.z =  final_control_angle*rotate_pid_control_kp*(angle_diff> 0.0 ? 1.0:-1.0);;
                            cmd_vel_msg.angular.z =  static_rotate_vel*(angle_diff> 0.0 ? 1.0:-1.0);;

                        }else{
                            double rotate_pid_inc = rotate_pid.calculate(angle_diff, 0.0);
                            PLOGD << "rotate_pid_inc: " << rotate_pid_inc << std::endl;

                            if(std::abs(rotate_pid_inc) < control_rotate_vel_tolerance || std::abs(angle_diff) < control_angle_tolerance){
                                rotate_pid_inc = 0.0;
                            }
                            cmd_vel_msg.angular.z =  rotate_pid_inc;

                        }
                    }

                    if(std::abs(control_target_in_base[0]) <detect_target_stop_update_dist){
                        cmd_vel_msg.angular.z =  0.0;

                    }




                    float target_pose_x = distance_diff * std::cos(marker_angle);


//            if(std::abs(angle_diff) < 0.02)
                    {

                        if(std::abs(target_pose_x) > final_control_dist){
                            cmd_vel_msg.linear.x =  final_control_dist*forward_pid_control_kp*(target_pose_x> 0.0 ? 1.0:-1.0);
                            cmd_vel_msg.linear.x =  static_forward_vel;

                        }else{
                            double forward_pid_inc = forward_pid.calculate(target_pose_x, 0.0);

                            PLOGD << "forward_pid_inc: " << forward_pid_inc << std::endl;
                            if(std::abs(forward_pid_inc) < control_forward_vel_tolerance || std::abs(target_pose_x) < control_dist_tolerance){
                                forward_pid_inc = 0.0;
                            }
                            cmd_vel_msg.linear.x =  forward_pid_inc;
                        }

                        debug_info_msg.x =target_pose_x;
                        debug_info_msg.y =marker_angle;

                        debug_info_pub.publish(debug_info_msg);

                    }



                    PLOGD << "cmd_vel_msg.angular.z : " <<   cmd_vel_msg.angular.z  << std::endl;
                    PLOGD << "cmd_vel_msg.linear.x : " <<  cmd_vel_msg.linear.x << std::endl;
                    if(!enable_control){
                        continue;
                    }

                    cmd_vel_pub.publish(cmd_vel_msg);

                    if(std::abs(target_pose_x)<control_dist_tolerance
//                && std::abs(angle_diff) < control_angle_tolerance
                            ){
                        PLOGD << "control finished test : " <<  control_finished_cnt << std::endl;

                        control_finished_cnt++;
                        if(control_finished_cnt > 10){
                            start_run = 0;
                        }
                    }

                }






            }else if(std::strcmp(mode.c_str(), MODE_SHELF2) == 0){

//                compute_leg_pose();

            }






        }else{
            suspend.sleep(sleep_time);

        }

    }

    if(enable_control&&(control_finished_cnt>0)){

        const char* final_cmd_vel = R"(
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
)";


        namespace sp = subprocess;
        auto p = sp::Popen(
                {"bash", "-c", final_cmd_vel}, sp::output{sp::PIPE},
                sp::error{sp::PIPE}, sp::defer_spawn{true});
        //   auto p = sp::Popen({"bash", "-c", cmd_string},
        //   sp::shell{true}, sp::defer_spawn{true});

        p.start_process();
    }


    return 0;
}