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


#include "json.hpp"

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



int main(int argc, char **argv) {



    //==== ros
    ros::init(argc, argv, "map_republisher");
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
    std::string base64Img_param = "base64Img";
    std::string returnTime_param = "returnTime";


    getParam(map_name_param, map_name);
    getParam(agv_sn_param, agv_sn);
    getParam(mqtt_server_ip_param, mqtt_server_ip);
    getParam(mqtt_server_port_param, mqtt_server_port);
    getParam(mqtt_topic_qos_param, mqtt_topic_qos);


    std::string mqtt_topic = "agv/map/ret/" + agv_sn;
    mqtt_client_id = mqtt_client_id + agv_sn;

    std::cout << agv_sn_param << " : " << agv_sn << std::endl;
    std::cout << map_name_param << " : " << map_name << std::endl;
    std::cout << mqtt_server_ip_param << " : " << mqtt_server_ip << std::endl;
    std::cout << mqtt_server_port_param << " : " << mqtt_server_port << std::endl;
    std::cout << mqtt_topic_qos_param << " : " << mqtt_topic_qos << std::endl;

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
    tf::StampedTransform transform;

    ROS_INFO("Waiting for tf transform.");

    std::string map_frame = "/map";
    std::string laser_frame = "/base_laser";

    while(!tf_transform_received)
    {

    }



    auto cb = [&](const sensor_msgs::LaserScanConstPtr &msg) {


        // get time
        // lookup tf
        try
        {
            tl_.waitForTransform(map_frame, msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
            tl_.lookupTransform(map_frame, msg->header.frame_id, ros::Time(0), transform);
            transform_.setOrigin(transform.getOrigin());
            transform_.setRotation(transform.getRotation());
            tf_transform_received = true;
            ROS_INFO("Received tf transform.");
            tf_transform_received = true;


        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
//            ros::Duration(1.0).sleep();
            tf_transform_received = false;
        }

        // transform points

        // get json

        if(tf_transform_received){




            j[returnTime_param] = getCurrentDateTime();
            j[base64Img_param] = img_data;

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