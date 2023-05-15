//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace legged {
using namespace ocs2;

class KalmanFilterEstimate : public StateEstimateBase {
 public:
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

  Eigen::Matrix<double, 3, 1> getvel_basefrane(void) override;
  Eigen::Matrix<double, 4, 1> get_localfoot_H(void) override;
  scalar_t get_terrain_angle(void) override;

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  void updateFromTopic();

  void updateFromOptiTrack();

  void callback(const nav_msgs::Odometry::ConstPtr& msg);
  void OptiTrack_callback(const geometry_msgs::PoseStamped::ConstPtr& msg); 

  nav_msgs::Odometry getOdomMsg();

  vector_t feetHeights_;  // 4 * 1
  Eigen::Matrix<double,4,1> feetHeights_read;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

 private:
  Eigen::Matrix<scalar_t, 18, 1> xHat_; // p_base, v_base, p_foot1234
  Eigen::Matrix<scalar_t, 12, 1> ps_; // 4个foot在base下的位置
  Eigen::Matrix<scalar_t, 12, 1> vs_; // 4个foot在base下的速度
  Eigen::Matrix<scalar_t, 18, 18> a_; // 状态矩阵
  Eigen::Matrix<scalar_t, 18, 18> q_; // process noise
  Eigen::Matrix<scalar_t, 18, 18> p_;
  Eigen::Matrix<scalar_t, 28, 28> r_; // measurement noise
  Eigen::Matrix<scalar_t, 18, 3> b_;  // 输入矩阵
  Eigen::Matrix<scalar_t, 28, 18> c_; // 观测矩阵

  // Topic
  ros::Subscriber sub_;
  ros::Subscriber OptiTrack_sub_; 
  ros::Publisher OptiTrack_odom_pub_;

  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::PoseStamped> OptiTrack_buffer_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;

  tf2::Transform OptiTrack_offset;
  tf2::Transform OptiTrack_realtime;

  bool topicUpdated_;
  bool optiTrackUpdated_;
  bool optiTrack_firstreceived;
};

}  // namespace legged
