//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <legged_msgs/leggedreference.h>

namespace legged {
using namespace ocs2;

double z_offset;
// double z_offset_last;

double terrain_pitch;

class TargetTrajectoriesPublisher final {
 public:

  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  // constructor
  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories, CmdToTargetTrajectories dummygoalToTargetTrajectories,CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        dummygoalToTargetTrajectories_(std::move(dummygoalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        tf2_(buffer_) 
  {
    //! ocs2 Trajectories publisher
    //   targetTrajectoriesPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(topicPrefix + "_mpc_target", 1, false);
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    //! ocs2 observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) 
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback); //! subscribe /legged_robot_mpc_observation

    //! rviz goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
      if (latestObservation_.time == 0.0) 
      {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try 
      {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } 
      catch (tf2::TransformException& ex) 
      {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);  //! publish /legged_robot_mpc_target
    };
    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);

    auto dummygoalCallback = [this](const geometry_msgs::Pose2D::ConstPtr& msg)
    {
      if (latestObservation_.time == 0.0)
      {
        return;
      }

      vector_t cmdGoal_x_y_theta = vector_t::Zero(3);
      cmdGoal_x_y_theta[0] = msg->x;
      cmdGoal_x_y_theta[1] = msg->y;
      cmdGoal_x_y_theta[2] = msg->theta;

      const auto trajectories = dummygoalToTargetTrajectories_(cmdGoal_x_y_theta, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };
    dummygoalSub_ = nh.subscribe<geometry_msgs::Pose2D>("/test/pose2D", 1, dummygoalCallback);

    //! robot_steering_tool cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) 
    {
      if (latestObservation_.time == 0.0) 
      {
        // std::cout << "enter here" << std::endl;
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);  //! /legged_robot_mpc_target
    };
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

    //! use elevation_planner_terrain_layer to get the position in z-axis
    auto z_offset_Callback = [this](const legged_msgs::leggedreference::ConstPtr& msg)
    {
      z_offset = msg->z_offset;
      terrain_pitch = msg->pitch_offset;  //! now we don't use pitch.
    };
    z_offset_Sub_ = nh.subscribe<legged_msgs::leggedreference>("/legged_reference_Topic", 1, z_offset_Callback);

  }

 private:
  CmdToTargetTrajectories goalToTargetTrajectories_, dummygoalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, goalSub_, dummygoalSub_, cmdVelSub_, z_offset_Sub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
