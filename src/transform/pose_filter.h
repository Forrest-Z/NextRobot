//
// Created by waxz on 23-2-4.
//

#ifndef SCAN_REPUBLISHER_POSE_FILTER_H
#define SCAN_REPUBLISHER_POSE_FILTER_H

#include "transform.h"
#include "common/clock_time.h"
namespace transform{

    struct MovementCheck {
        float no_move_translation_epsilon = 5e-3;
        float no_move_rotation_epsilon = 5e-3;
        float final_no_move_translation_epsilon = 1e-2;
        float final_no_move_rotation_epsilon = 1e-2;

        float move_translation_epsilon = 0.1;
        float move_rotation_epsilon = 0.05;

        float no_move_check_ms = 200.0;
        transform::Transform2d last_pose_inv;
        transform::Transform2d last_move_pose_inv;

        transform::Transform2d start_check_pose_inv;

        transform::Transform2d movement;

        common::Time last_time;

        bool start_check = false;
        bool still = false;
        long move_flag = 0;
        long move_flag_last = 0;
        long update_flag = 0;

        void reset(){

            start_check = false;
            still = false;

            move_flag = 0;
            move_flag_last = 0;
            update_flag = 0;
        }
        // trigger at move or first frame
        bool checkMoveTrigger(const transform::Transform2d &new_pose) {
            movement = last_move_pose_inv * new_pose;

            bool is_move = std::abs(movement.x()) > move_translation_epsilon
                           || std::abs(movement.y()) > move_translation_epsilon
                           || std::abs(movement.yaw()) > move_rotation_epsilon;

            if(is_move|| (update_flag == 0)){
                last_move_pose_inv = new_pose.inverse();
                move_flag++;
            }

//        PLOGD << "movement " << movement << "\n move_flag " << move_flag << "\n move_flag_last " << move_flag_last << std::endl;

            update_flag++;
            return move_flag!=move_flag_last;

        }
        bool isMoveTriggered(){
            return move_flag!=move_flag_last;
        }
        bool checkStill(const transform::Transform2d &new_pose) {

            movement = last_pose_inv * new_pose;

//        PLOGD << "new_pose:\n" << new_pose << std::endl;
//        PLOGD << "movement:\n" << movement << std::endl;

            bool no_move = std::abs(movement.x()) < no_move_translation_epsilon &&
                           std::abs(movement.y()) < no_move_translation_epsilon
                           && std::abs(movement.yaw()) < no_move_rotation_epsilon;

            movement = start_check_pose_inv * new_pose;
//        PLOGD << "movement:\n" << movement << std::endl;

            no_move = no_move && std::abs(movement.x()) < final_no_move_translation_epsilon &&   std::abs(movement.y()) < final_no_move_translation_epsilon  && std::abs(movement.yaw()) < final_no_move_rotation_epsilon;

            if ((no_move && !start_check) || !no_move) {
                last_time = common::FromUnixNow();
                start_check_pose_inv = new_pose.inverse();
            }
            last_pose_inv = new_pose.inverse();

            start_check = no_move;
            still = common::ToMillSeconds(common::FromUnixNow() - last_time) > no_move_check_ms;


            return still;

        }

        bool isStill()  {
//        still = common::ToMillSeconds(common::FromUnixNow() - last_time) > no_move_check_ms;

            return still;
        }

        float getStillTime(){
            return common::ToMillSeconds(common::FromUnixNow() - last_time);
        }


    };

    class PoseFilter{
    private:
        int max_len = 5;
        std::deque<transform::Transform2d> buffer;

    public:
        PoseFilter(int t_max_len = 5):max_len(t_max_len){ }
        void setLen(int t_max_len){
            max_len = t_max_len;
        }
        transform::Transform2d add(const transform::Transform2d& t_pose){
            if(buffer.size() > max_len){
                buffer.pop_back();
            }
            buffer.emplace_front(t_pose);

            float x = 0.0, y = 0.0 ,yaw = 0.0;
            float init_yaw = t_pose.yaw();
            transform::Transform2d relative_pose;
            for(int i = 1 ; i < buffer.size();i++){
                x += buffer[i].x();
                y += buffer[i].y();
                yaw += angle_normalise(buffer[i].yaw(), init_yaw) ;
            }
            x /= buffer.size();
            y /= buffer.size();
            yaw /= buffer.size();

            relative_pose.set(x,y,yaw);

            return relative_pose;
        }
    };
}
#endif //SCAN_REPUBLISHER_POSE_FILTER_H