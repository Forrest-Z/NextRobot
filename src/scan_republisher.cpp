//
// Created by waxz on 9/5/22.
//

#include <iostream>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "xmlrpcpp/XmlRpc.h"

#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


//#include <mosquitto.h>
//#include <mosquittopp.h>
#include "message/MqttClient.h"


#include "nlohmann/json.hpp"

#include "common/signal_handler.h"
#include "../include/common/signal_handler.h"

template<typename T>
bool getParam(const std::string &name, T &value) {
    if (name.empty()) {
        std::cerr << "ros get xml value, empty name: " << name << std::endl;
        return false;
    }
    std::string private_name = name[0] != '/' ? ros::this_node::getName() + "/" + name : name;
    if (ros::param::has(private_name)) {
        std::cerr << "ros get xml value, has name: " << private_name << std::endl;
        ros::param::get(private_name, value);
        return true;
    } else {
        std::cerr << "ros get xml value,do not  has name: " << private_name << std::endl;
        return false;
    }
    return true;
}


#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
std::string getCurrentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);

    return buf;
}
template <typename T>
void lin_space(T start, T inc, int n, std::vector<T>& data){
    data.resize(n);
    std::for_each(data.begin(),data.end(),[&, inc = inc,start = start](auto& e)mutable {
        e = start;
        start += inc;
    });
}

template <typename F, typename ...Args>
void func_map_v3( const  F& func,   Args& ... args){
    size_t arr[] = {  args.size()...  };
    const int sz = sizeof...(args);
    if(sz > 0){
        size_t N= arr[0];
        for(int i = 1 ; i < sz;i++){
            if(arr[i] == 0 || arr[i]%N != 0){
                std::cout << __FUNCTION__  << " check size error: " << arr[i]<< "\n";
            }
        }
        for(int i = 0; i < N;i++){
            func(i,args[0]...  );
        }
    }
}


template <typename F, typename ...Args>
void func_map_v4( const  F& func,   Args& ... args){
    size_t arr[] = {  args.size()...  };
    const int sz = sizeof...(args);
    if(sz > 0){
        size_t N= arr[0];
        for(int i = 1 ; i < sz;i++){
            if(arr[i] == 0 || arr[i]%N != 0){
                std::cout << __FUNCTION__  << " check size error: " << arr[i]<< "\n";
            }
        }
        for(int i = 0; i < N;i++){
            func(args[i]...  );
        }
    }
}

struct Transform2d{
    float matrix[3][3] = {};
    Transform2d(float x=0.0, float y=0.0,float yaw=0.0){
        set(x,y,yaw);
    }
    void set(float x=0.0, float y=0.0,float yaw=0.0){
        // Calculate rotation about z axis
        /*
                 cos(yaw),   -sin(yaw),      0,
                 sin(yaw),   cos(yaw),       0,
                 0,          0,              1
             */
        matrix[0][0] = cos(yaw);
        matrix[0][1]  = -sin(yaw);
        matrix[1][0] = sin(yaw);
        matrix[1][1]  = cos(yaw);

        matrix[0][2]  = x;
        matrix[1][2]  = y;

        matrix[2][0]  = 0.0;
        matrix[2][1]  = 0.0;
        matrix[2][2]  = 1.0;

    }
    Transform2d operator*(const Transform2d& rhv){
        Transform2d result;

        auto& a = this->matrix;
        auto& b = rhv.matrix;
        auto& c = result.matrix;
        // Calculate the j-th column of the result in-place (in B) using the helper array
        for(int i=0 ; i<3 ; i++)
        {
            for(int j=0 ; j<3 ; j++)
            {

                c[i][j]=0;
                for(int k=0 ; k<3 ; k++)
                {
                    c[i][j]+=a[i][k]*b[k][j];
                    //--^-- should be k
                }
            }
        }
        return result;

    }
    void mul(const std::vector<float>& points, std::vector<float>& result){

        /*
     r00 r01 r02 tx     x0        x1
     r10 r11 r12 ty  X  y0   =>   y1
     r20 r21 r22 tz     z0        z1
     0   0   0   1      1         1
    */

        if(points.size()%2 != 0 ){
            std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

        }
        result.resize(points.size());

        float r00 = this->matrix[0][0];
        float r01 = this->matrix[0][1];

        float r10 = this->matrix[1][0];
        float r11 = this->matrix[1][1];

        float tx = this->matrix[0][2];
        float ty = this->matrix[1][2];

        int n_dim = points.size()/2;

        const float *p_data_x = &(points[0]);
        const float *p_data_y = p_data_x + n_dim;

        float *p_x = &(result[0]);
        float *p_y = p_x + n_dim;

        for (int i = 0; i < n_dim; i++) {
            p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
            p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
        }

    }
    Transform2d operator*(const Transform2d& rhv )const{
        Transform2d transform;
        auto & mat  =this->matrix;
        auto & mat_rhv  =rhv.matrix;
        auto & mat_mul  =transform.matrix;
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++)
                mat_mul[i][j] += mat[i][j] * mat_rhv[j][i] ;
        }

        return transform;

    }
    Transform2d inverse() const{
        Transform2d transform_inv;
        float determinant = 0;

        auto & mat  =this->matrix;
        //finding determinant
        for(int i = 0; i < 3; i++)
            determinant += (mat[0][i] * (mat[1][(i+1)%3] * mat[2][(i+2)%3] - mat[1][(i+2)%3] * mat[2][(i+1)%3]));

        auto & mat_inv  =transform_inv.matrix;
        float determinant_inv = 1.0f/determinant;

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++)
                mat_inv[i][j]= ((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))* determinant_inv ;

        }

        return transform_inv;

    }

};
std::ostream& operator <<(std::ostream& out,const Transform2d& rhv ){
    out << "Transform2d:\n";
//    out.unsetf ( std::ios::floatfield );                // floatfield not set
    out.precision(5);
    out.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed

    out << "[" << rhv.matrix[0][0] << ", " << rhv.matrix[0][1] << ", " << rhv.matrix[0][2]<<"\n"
        << " " << rhv.matrix[1][0] << ", " << rhv.matrix[1][1] << ", " << rhv.matrix[1][2]<<"\n"
        <<" " << rhv.matrix[2][0] << ", " << rhv.matrix[2][1] << ", " << rhv.matrix[2][2]<<"]\n"
        << std::endl;
    out.unsetf ( std::ios::floatfield );                // floatfield not set

    return out;
}


struct ScanToPoints{

    std::vector<float> angle_buffer;
    std::vector<float> cos_angle_buffer;
    std::vector<float> sin_angle_buffer;
    std::vector<float> local_xy_points;
    std::vector<float> global_xy_points;
    std::vector<std::array<float,3>> global_xy_points_vec;
    Transform2d transform2D;

    bool init_done = false;
    void getLocalPoints( const sensor_msgs::LaserScanConstPtr  & msg){
        if(init_done){

            auto ranges =  std::remove_const_t<std::vector<float>>(msg->ranges)  ;
            auto intensities = msg->intensities;

            auto range_max = msg->range_max +1;

            func_map_v4([&](auto&a){
                a = std::isnormal(a) ? a:range_max ;
            }, ranges);

            for(int i = 0 ; i < ranges.size();i++){
                local_xy_points[i+i] = ranges[i]*cos_angle_buffer[i];
                local_xy_points[i+i +1] = ranges[i]*sin_angle_buffer[1];

            }
            if(intensities.size() == ranges.size()){
                for(int i = 0 ; i < ranges.size();i++){
                    global_xy_points_vec[i][2] =  intensities[i]  ;
                }
            }
        }else{
            lin_space(msg->angle_min,msg->angle_increment,msg->ranges.size(),angle_buffer);
            cos_angle_buffer.resize(angle_buffer.size());
            sin_angle_buffer.resize(angle_buffer.size());
            local_xy_points.resize(2*angle_buffer.size());
            global_xy_points.resize(2*angle_buffer.size());
            global_xy_points_vec.resize(angle_buffer.size(),std::array<float,3>({0.0,0.0,0.0}));

            func_map_v4([](auto&a, auto&b, auto&c){
                b= cos(a);
                c = sin(a);
                }, angle_buffer,cos_angle_buffer,sin_angle_buffer);
            init_done = true;

        }

    }
    void getGlobalPoints(float x, float y, float yaw){
        transform2D.set(x,y,yaw);
        transform2D.mul(local_xy_points,global_xy_points);

        int N= global_xy_points_vec.size();


        for(int i = 0 ; i < N ;i++){
            global_xy_points_vec[i][0] = global_xy_points[i+i];
            global_xy_points_vec[i][1] = global_xy_points[i+i+1];
        }

    }

    void getGlobalPoints(){

        int N= global_xy_points_vec.size();

        for(int i = 0 ; i < N ;i++){
            global_xy_points_vec[i][0] = local_xy_points[i+i];
            global_xy_points_vec[i][1] = local_xy_points[i+i+1];
        }

    }


            };

int main(int argc, char **argv) {



    //==== ros
    ros::init(argc, argv, "scan_republisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    XmlRpc::XmlRpcValue xmlvalue;

    std::string mqtt_server_ip = "10.23.0.125";
    int mqtt_server_port = 1883;
    int mqtt_topic_qos = 0;
    std::string mqtt_user_name = "user_1";
    std::string mqtt_passward = "user_1";
    std::string mqtt_client_id = "agv-";
    std::string mqtt_will_topic;
    std::string mqtt_will_message;
    int mqtt_keep_alive = 5;


    bool mqtt_debug = true;

    std::string mqtt_server_ip_param = "mqtt_server_ip";
    std::string mqtt_server_port_param = "mqtt_server_port";
    std::string mqtt_topic_qos_param = "mqtt_topic_qos";


    std::string map_name = "map_0";
    std::string map_name_param = "mapName";
    std::string agv_sn = "q0000001";
    std::string agv_sn_param = "agvSn";
    std::string laser_param = "message";
    std::string returnTime_param = "returnTime";

    std::string fixed_frame = "/map";
    std::string fixed_frame_param = "fixed_frame";


    getParam(map_name_param, map_name);
    getParam(agv_sn_param, agv_sn);
    getParam(mqtt_server_ip_param, mqtt_server_ip);
    getParam(mqtt_server_port_param, mqtt_server_port);
    getParam(mqtt_topic_qos_param, mqtt_topic_qos);
    getParam(fixed_frame_param, fixed_frame);


    std::string mqtt_topic = "agv/laser/ret/" + agv_sn;
    mqtt_client_id = mqtt_client_id + agv_sn;

    std::cout << agv_sn_param << " : " << agv_sn << std::endl;
    std::cout << map_name_param << " : " << map_name << std::endl;
    std::cout << mqtt_server_ip_param << " : " << mqtt_server_ip << std::endl;
    std::cout << mqtt_server_port_param << " : " << mqtt_server_port << std::endl;
    std::cout << mqtt_topic_qos_param << " : " << mqtt_topic_qos << std::endl;
    std::cout << fixed_frame_param << " : " << fixed_frame << std::endl;

    std::cout << "mqtt_topic" << " : " << mqtt_topic << std::endl;


    mosqpp::lib_init();

    message::MqttClient MqttInstance(mqtt_client_id.c_str(), true);

//    MqttClient& MqttInstance = MqttClient::Singleton::getInstance<MqttClient>(client_id,true);
    MqttInstance.message_callback_func = [&](const char *topic, const char *playload) {

        std::cout << "message_callback_func : " << topic << ", " << playload << std::endl;

        return 1;
    };

//    mqtt_will_topic = "online";
//    mqtt_will_message = "die";

    MqttInstance.addSubOpt("hello", 0);
    MqttInstance.addSubOpt("hello2", 0);
    MqttInstance.addSubOpt(mqtt_topic.c_str(), 0);

    MqttInstance.listSubOpts();

    if (!mqtt_will_topic.empty() and !mqtt_will_message.empty()) {
        MqttInstance.will_set(mqtt_will_topic.c_str(), mqtt_will_message.size(), mqtt_will_message.c_str());
    }

    MqttInstance.connect_async(mqtt_server_ip.c_str(), mqtt_server_port, mqtt_keep_alive);

    int rc = 0;

    MqttInstance.loop_start();



    // ros topic


    nlohmann::json j;

    j[agv_sn_param] = agv_sn;
    j[map_name_param] = map_name;


    std::string img_data;


    std::string json_string;


    bool show_image = false;
    std::cout << "start ros loop " << std::endl;
    MqttInstance.reconnect_delay_set(1, 2, false);

    tf::TransformListener tl_;

    tf::Transform transform_;
    tf::Transform identity_transform_;

    bool tf_transform_received = false;
    bool scan_received = false;

    tf::StampedTransform transform;

    ROS_INFO("Waiting for tf transform.");


    ScanToPoints scan_handler;
    Transform2d transform2D;



    auto cb = [&]( const sensor_msgs::LaserScanConstPtr &msg) {

        scan_handler.getLocalPoints(msg);


        // get time
        // lookup tf
        if(fixed_frame != msg->header.frame_id){
            try {
                tl_.waitForTransform(fixed_frame, msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
                tl_.lookupTransform(fixed_frame, msg->header.frame_id, ros::Time(0), transform);
                transform_.setOrigin(transform.getOrigin());
                transform_.setRotation(transform.getRotation());
                tf_transform_received = true;
                ROS_INFO("Received tf transform.");
                tf_transform_received = true;
                scan_handler.getGlobalPoints(transform_.getOrigin().x(),transform_.getOrigin().y(),  tf::getYaw(transform_.getRotation()));

            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
//            ros::Duration(1.0).sleep();
                tf_transform_received = false;
            }



        }else{
            tf_transform_received = true;
            scan_handler.getGlobalPoints();

        }


        // transform points

        // get json

        if (tf_transform_received) {


            j[returnTime_param] = getCurrentDateTime();
            j[laser_param] = scan_handler.global_xy_points_vec;

            json_string = j.dump();
            MqttInstance.publish(nullptr, mqtt_topic.c_str(), json_string.size(), json_string.c_str(), mqtt_topic_qos);

        }


    };
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, cb);


    while (ros::ok()) {

        rc = MqttInstance.loop();
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "Error mosquitto_loop_misc: %s\n", mosquitto_strerror(rc));
        }
        if (rc == MOSQ_ERR_NO_CONN) {
            std::cout << " MOSQ_ERR_NO_CONN : " << rc << std::endl;
        }

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1000ms);
        ros::spinOnce();


    }
//    mosqpp::lib_cleanup();

    return 0;
}