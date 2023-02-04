//
// Created by waxz on 23-2-4.
//

#include "RvizInteractiveControlTool.h"
namespace ros_tool {

    void RvizInteractiveControlTool::setCallBack(std::function<void(const geometry_msgs::Pose &)> &&t_cb) {
        m_callback = std::move(t_cb);
    }

    void RvizInteractiveControlTool::setInitPose(float x, float y, float yaw) {
        pose_.position.x = x;
        pose_.position.y = y;
        pose_.position.z = 1.0;

        math::yaw_to_quaternion(yaw, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);

    }

    geometry_msgs::Pose &RvizInteractiveControlTool::getPose() {
        return pose_;
    }

    RvizInteractiveControlTool::RvizInteractiveControlTool(const std::string &t_parent_frame, const std::string &t_target_frame) :
            parent_frame_(t_parent_frame),
            frame_(t_target_frame) {}

    void RvizInteractiveControlTool::start(const std::string& name, float scale, bool pub_tf) {

        if(is_start){
            std::cout << "RvizInteractiveControlTool is already running" << std::endl;

            return;
        }

        char topic_name[100];
        std::cout << "interactive_markers name = " << name << std::endl;

        sprintf(topic_name,"%s/interactive_tf",name.c_str());
        std::cout << "interactive_markers topic_name = " << topic_name << std::endl;
        server_.reset(new interactive_markers::InteractiveMarkerServer(topic_name));

        // TODO(lucasw) need way to get parameters out- tf echo would work

        int_marker_.header.frame_id = parent_frame_;
        // http://answers.ros.org/question/262866/interactive-marker-attached-to-a-moving-frame/
        // putting a timestamp on the marker makes it not appear
        // int_marker_.header.stamp = ros::Time::now();
        int_marker_.name = "interactive_tf";
        int_marker_.description = "control a tf with 6dof";
        int_marker_.pose = pose_;
        int_marker_.scale = 1.0;

        {
            visualization_msgs::InteractiveMarkerControl control;

            // TODO(lucasw) get roll pitch yaw and set as defaults

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "rotate_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//            int_marker_.controls.push_back(control);
            control.name = "move_x";
            // TODO(lucasw) how to set initial values?
            // double x = 0.0;
            // ros::param::get("~x", x);
            // control.pose.position.x = x;
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker_.controls.push_back(control);
            // control.pose.position.x = 0.0;

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//            int_marker_.controls.push_back(control);
            control.name = "move_y";
            // double y = 0.0;
            // control.pose.position.z = ros::param::get("~y", y);
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker_.controls.push_back(control);
            // control.pose.position.y = 0.0;

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker_.controls.push_back(control);
            control.name = "move_z";
            // double z = 0.0;
            // control.pose.position.z = ros::param::get("~z", z);
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//            int_marker_.controls.push_back(control);
            // control.pose.position.z = 0.0;


        }

        server_->insert(int_marker_);
        server_->setCallback(int_marker_.name,
                             boost::bind(&RvizInteractiveControlTool::processFeedback, this, 0, boost::placeholders::_1));
        // server_->setCallback(int_marker_.name, testFeedback);

        server_->applyChanges();

        if (pub_tf) {
//            tf_timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&RvizInteractiveControlTool::updateTf, this, 0, boost::placeholders::_1));
        }
        is_start = true;
        std::cout << "RvizInteractiveControlTool start" << std::endl;
    }

    RvizInteractiveControlTool::~RvizInteractiveControlTool() {
        server_.reset();
    }
#if 0
    void RvizInteractiveControlTool::updateTf(int, const ros::TimerEvent &event) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose_.position.x, pose_.position.y, pose_.position.z));
        transform.setRotation(tf::Quaternion(pose_.orientation.x,
                                             pose_.orientation.y,
                                             pose_.orientation.z,
                                             pose_.orientation.w));
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                               parent_frame_, frame_));
    }
#endif
    void RvizInteractiveControlTool::processFeedback(
            unsigned ind,
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        ROS_DEBUG_STREAM(feedback->header.frame_id);
        pose_ = feedback->pose;
        if (m_callback) {
            m_callback(pose_);
        }
        ROS_DEBUG_STREAM(feedback->control_name);
        ROS_DEBUG_STREAM(feedback->event_type);
        ROS_DEBUG_STREAM(feedback->mouse_point);
        // TODO(lucasw) all the pose changes get handled by the server elsewhere?
        server_->applyChanges();
    }

}