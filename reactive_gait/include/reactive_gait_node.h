#include "ros/ros.h"
#include <time.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "ocs2_msgs/mode_schedule.h"

class Reactive_Gait_Node
{
private:
    /* data */


public:
    ros::Publisher gait_pub;
    ros::Subscriber darknet_sub;

    ocs2_msgs::mode_schedule trot_mode_msg;
    ocs2_msgs::mode_schedule walk_mode_msg;

    clock_t last_walk_clock;
    clock_t now_clock;

    int current_mode; // 0: trot, 1: walk 2:stance

    Reactive_Gait_Node(ros::NodeHandle nh)
    {
        current_mode = 2;
        last_walk_clock = 0;
        gait_pub = nh.advertise<ocs2_msgs::mode_schedule>("/legged_robot_mpc_mode_schedule", 1);

                    
        trot_mode_msg.eventTimes = {0.0, 0.3, 0.6};
        trot_mode_msg.modeSequence = {9, 6};   

        // walk_mode_msg.eventTimes = {0.0, 0.3, 0.6, 0.9, 1.2};
        // walk_mode_msg.modeSequence = {13, 7, 14, 11}; 

        walk_mode_msg.eventTimes = {0.0, 0.2, 0.3, 0.5, 0.7, 0.8, 1.0};
        walk_mode_msg.modeSequence = {13, 5, 7, 14, 10, 11}; 

        auto reactive_gait_callback = [this](const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
        {
            // ROS_INFO("I heard: [%d]", msg->bounding_boxes[0].id);  

            // if()

            // if( (msg->bounding_boxes[0].id == 59 || msg->bounding_boxes[1].id == 59 || msg->bounding_boxes[2].id == 59) )       // 0 41 
            // {
            //     if(current_mode != 1)
            //     {
            //         gait_pub.publish(stance_mode_msg);
            //         ROS_INFO("switch to walk gait");  
            //     }
            //     last_stair_clock = ros::Time::now().toSec();
            //     current_mode = 1;
            // }

            int person_num = 0;
            for(int i = 0;i<=msg->bounding_boxes.size(); i++)
            {
                if(msg->bounding_boxes[i].id == 0)  // person
                {
                    person_num ++;
                }
                else if(msg->bounding_boxes[i].id == 41)
                {
                    person_num = 2;
                }
            }

            if(current_mode != 1 && person_num>1)
            {
                gait_pub.publish(walk_mode_msg);
                ROS_INFO("switch to walk gait");  
                last_walk_clock = ros::Time::now().toSec();
                current_mode = 1;
            }


        };
        darknet_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, reactive_gait_callback);

    }

};
