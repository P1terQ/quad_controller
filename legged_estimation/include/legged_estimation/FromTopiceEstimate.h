//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/StateEstimateBase.h"

#include "ocs2_robotic_tools/common/RotationTransforms.h"

#include <realtime_tools/realtime_buffer.h>

#pragma once
namespace legged {
using namespace ocs2;

class MovingWindowFilter {
 public:

  MovingWindowFilter() {}

  MovingWindowFilter(int window_size) : window_size_(window_size) {
    assert(window_size_ > 0);
    sum_ = 0.0;
    correction_ = 0.0;
  }

  // Computes the moving window average.
  double CalculateAverage(double new_value) {
    if (value_deque_.size() < window_size_) {
      // pass
    } else {
      // The left most value needs to be subtracted from the moving sum first.
      UpdateNeumaierSum(-value_deque_.front());
      value_deque_.pop_front();
    }
    // Add the new value.
    UpdateNeumaierSum(new_value);
    value_deque_.push_back(new_value);

    return (sum_ + correction_) / double(window_size_);
  }

  std::deque<double> GetValueQueue() {
    return value_deque_;
  }
 private:
  int window_size_;
  double sum_, correction_;
  std::deque<double> value_deque_;

  // Updates the moving window sum using Neumaier's algorithm.
  //
  // For more details please refer to:
  // https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
  void UpdateNeumaierSum(double value) {
    double new_sum = sum_ + value;
    if (std::abs(sum_) >= std::abs(value)) {
      // If previous sum is bigger, low-order digits of value are lost.
      correction_ += (sum_ - new_sum) + value;
    } else {
      correction_ += (value - new_sum) + sum_;
    }
    sum_ = new_sum;
  }
};

class FromTopicStateEstimate : public StateEstimateBase {
 public:
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics);

  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                 const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                 const matrix3_t& linearAccelCovariance) override{};

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

  Eigen::Matrix<double, 3, 1> getvel_basefrane(void) override;
  Eigen::Matrix<double, 4, 1> get_localfoot_H(void) override;
  scalar_t get_terrain_angle(void) override;

 private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;

  Eigen::Matrix<double, 3, 1> vel_baseframe{0,0,0};

  scalar_t z_first_offset;
  bool first_received;

  Eigen::Matrix<double,4,1> feetHeights_read;

  scalar_t terrain_angle;
  Eigen::Matrix<double, 3, 4> foot_pos_recent_contact;
  MovingWindowFilter terrain_angle_filter;
  MovingWindowFilter recent_contact_x_filter[4];
  MovingWindowFilter recent_contact_y_filter[4];
  MovingWindowFilter recent_contact_z_filter[4];

  ros::Publisher odom_navi_pub;
};



}  // namespace legged
