#include "ros/ros.h"
#include "reactive_gait_node.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "reactive_gait_node");

  ros::NodeHandle n;

  Reactive_Gait_Node dummy_node(n);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {

    // std::cout << "last_clock: " << dummy_node.last_stair_clock << std::endl;
    dummy_node.now_clock = ros::Time::now().toSec();
    // std::cout << "now_clock" << dummy_node.now_clock << std::endl;

    if(dummy_node.now_clock - dummy_node.last_walk_clock >= 2 && dummy_node.current_mode!=0)
    {
    std::cout << "last_clock: " << dummy_node.last_walk_clock << std::endl;
    std::cout << "now_clock: " << dummy_node.now_clock << std::endl;

      ROS_INFO("switch to trot gait"); 
      dummy_node.gait_pub.publish(dummy_node.trot_mode_msg);
      dummy_node.current_mode = 0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}


