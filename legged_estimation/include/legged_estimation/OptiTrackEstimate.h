#include "legged_estimation/StateEstimateBase.h"

#include "ocs2_robotic_tools/common/RotationTransforms.h"

#include <realtime_tools/realtime_buffer.h>

#include "geometry_msgs/PoseStamped.h"

#pragma once
namespace legged {
using namespace ocs2;

class OptiTrackStateEstimate : public StateEstimateBase {
 public:
  OptiTrackStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics);

  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                 const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                 const matrix3_t& linearAccelCovariance) override{};

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

  Eigen::Matrix<double, 3, 1> getvel_basefrane(void) override;
  Eigen::Matrix<double, 4, 1> get_localfoot_H(void) override;
  scalar_t get_terrain_angle(void) override;

 private:
  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  ros::Subscriber OptiTrack_sub_;



};



}  // namespace legged
