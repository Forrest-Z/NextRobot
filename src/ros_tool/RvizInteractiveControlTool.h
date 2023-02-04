//
// Created by waxz on 23-2-4.
//

#ifndef RVIZINTERACTIVECONTROLTOOL_H
#define RVIZINTERACTIVECONTROLTOOL_H


#include <memory>
#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>

#include "math/math_basic.h"

namespace ros_tool{

    class RvizInteractiveControlTool {
//        ros::NodeHandle nh_;
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

        void processFeedback(unsigned ind, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

        visualization_msgs::InteractiveMarker int_marker_;

        geometry_msgs::Pose pose_;
        std::string parent_frame_;
        std::string frame_;

        std::function<void(const geometry_msgs::Pose &)> m_callback;
        bool is_start = false;

    public:
        RvizInteractiveControlTool(const std::string &t_parent_frame, const std::string &t_target_frame);

        ~RvizInteractiveControlTool();

        void start(const std::string& name, float scale = 1.0, bool pub_tf = false);

        void setInitPose(float x, float y, float yaw);

        geometry_msgs::Pose &getPose();

        void setCallBack(std::function<void(const geometry_msgs::Pose &)> &&);
    };
}

#endif //RVIZINTERACTIVECONTROLTOOL_H
