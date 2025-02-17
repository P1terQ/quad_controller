//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/LinearKalmanFilter.h"

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {

double last_time_get;

KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics), tfListener_(tfBuffer_), topicUpdated_(false) 
{
  xHat_.setZero();
  ps_.setZero();
  vs_.setZero();
  a_.setZero();
  a_.block(0, 0, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(3, 3, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(6, 6, 12, 12) = Eigen::Matrix<scalar_t, 12, 12>::Identity();
  b_.setZero();

  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
  c1 << Eigen::Matrix<scalar_t, 3, 3>::Identity(), Eigen::Matrix<scalar_t, 3, 3>::Zero();
  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
  c2 << Eigen::Matrix<scalar_t, 3, 3>::Zero(), Eigen::Matrix<scalar_t, 3, 3>::Identity();
  c_.setZero();
  c_.block(0, 0, 3, 6) = c1;
  c_.block(3, 0, 3, 6) = c1;
  c_.block(6, 0, 3, 6) = c1;
  c_.block(9, 0, 3, 6) = c1;
  c_.block(0, 6, 12, 12) = -Eigen::Matrix<scalar_t, 12, 12>::Identity();
  c_.block(12, 0, 3, 6) = c2;
  c_.block(15, 0, 3, 6) = c2;
  c_.block(18, 0, 3, 6) = c2;
  c_.block(21, 0, 3, 6) = c2;
  c_(27, 17) = 1.0;
  c_(26, 14) = 1.0;
  c_(25, 11) = 1.0;
  c_(24, 8) = 1.0;
  p_.setIdentity();
  p_ = 100. * p_;
  q_.setIdentity();
  r_.setIdentity();
  feetHeights_.setZero(4);
  feetHeights_read.setZero();
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  world2odom_.setRotation(tf2::Quaternion::getIdentity());
  
  sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);
  
  OptiTrack_sub_ = ros::NodeHandle().subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/dog/pose", 10, &KalmanFilterEstimate::OptiTrack_callback, this);
  optiTrackUpdated_ = false;
  optiTrack_firstreceived = true;

  OptiTrack_odom_pub_ = ros::NodeHandle().advertise<geometry_msgs::Vector3>("/test/OptiTrack_Odom", 1);
  // odom_navi_pub = ros::NodeHandle().advertise<geometry_msgs::Vector3>("/odom_navi", 1);

  last_time_get = ros::Time::now().toSec();
}

vector_t KalmanFilterEstimate::update(const ros::Time& time, const ros::Duration& period) 
{
  scalar_t dt = period.toSec();

  //! 状态转移矩阵
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();

  //! 输入矩阵
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();

  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(6, 6, 12, 12) = dt * Eigen::Matrix<scalar_t, 12, 12>::Identity();

  //! use Pinocchio Interface get latest footpos & footvel
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  size_t actuatedDofNum = info_.actuatedDofNum;

  vector_t qPino(info_.generalizedCoordinatesNum);
  vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = rbdState_.head<3>();  //! Only set orientation, let position in origin.
  qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3),
      rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);

  const auto eePos = eeKinematics_->getPosition(vector_t());  //! footpos in base frame
  const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());  //! footori in base frame

  //! covariance matrix
  Eigen::Matrix<scalar_t, 18, 18> q = Eigen::Matrix<scalar_t, 18, 18>::Identity();
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
  q.block(6, 6, 12, 12) = q_.block(6, 6, 12, 12) * footProcessNoisePosition_;

  Eigen::Matrix<scalar_t, 28, 28> r = Eigen::Matrix<scalar_t, 28, 28>::Identity();
  r.block(0, 0, 12, 12) = r_.block(0, 0, 12, 12) * footSensorNoisePosition_;
  r.block(12, 12, 12, 12) = r_.block(12, 12, 12, 12) * footSensorNoiseVelocity_;
  r.block(24, 24, 4, 4) = r_.block(24, 24, 4, 4) * footHeightSensorNoise_;

  for (int i = 0; i < 4; i++) 
  {
    int i1 = 3 * i;

    int qIndex = 6 + i1;
    int rIndex1 = i1;
    int rIndex2 = 12 + i1;
    int rIndex3 = 24 + i;
    bool isContact = contactFlag_[i];

    //! update covariance matrix based on contact states
    scalar_t high_suspect_number(100);
    q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
    r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
    r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
    r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

    ps_.segment(3 * i, 3) = -eePos[i];
    ps_.segment(3 * i, 3)[2] += footRadius_;
    vs_.segment(3 * i, 3) = -eeVel[i];
  }

  vector3_t g(0, 0, -9.81);
  vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;

  Eigen::Matrix<scalar_t, 28, 1> y;
  y << ps_, vs_, feetHeights_;

  xHat_ = a_ * xHat_ + b_ * accel;  //! Update estimation

  Eigen::Matrix<scalar_t, 18, 18> at = a_.transpose();
  Eigen::Matrix<scalar_t, 18, 18> pm = a_ * p_ * at + q;
  Eigen::Matrix<scalar_t, 18, 28> cT = c_.transpose();
  Eigen::Matrix<scalar_t, 28, 1> yModel = c_ * xHat_;
  Eigen::Matrix<scalar_t, 28, 1> ey = y - yModel;
  Eigen::Matrix<scalar_t, 28, 28> s = c_ * pm * cT + r;

  Eigen::Matrix<scalar_t, 28, 1> sEy = s.lu().solve(ey);
  xHat_ += pm * cT * sEy;

  Eigen::Matrix<scalar_t, 28, 18> sC = s.lu().solve(c_);
  p_ = (Eigen::Matrix<scalar_t, 18, 18>::Identity() - pm * cT * sC) * pm;

  Eigen::Matrix<scalar_t, 18, 18> pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) 
  {
    p_.block(0, 2, 2, 16).setZero();
    p_.block(2, 0, 16, 2).setZero();
    p_.block(0, 0, 2, 2) /= 10.;
  }

  //! 一般这个都是false,不会接收到tracking_camera话题
  if (topicUpdated_) 
  {
    updateFromTopic();
    topicUpdated_ = false;
  }

  if(optiTrackUpdated_)
  {
    std::cout << "Update from OptiTrack" << std::endl;
    updateFromOptiTrack();
    optiTrackUpdated_ = false;
  }
  
  // for (size_t i = 0; i < 4; ++i) 
  // {
  //   if (contactFlag_[i]) 
  //   { //? 不知道这个高度到底是在那更新的。 
  //     feetHeights_read[i] = xHat_(6 + i * 3 + 2);   // 更新feet_height
  //   }
  // }
  
  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

  //! topicUpdate里边也会干同样的事
  auto odom = getOdomMsg();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";

  //todo 
  publishMsgs(odom);

  // last_time_get = ros::Time::now().toSec();
  // if(ros::Time::now().toSec() - last_time_get > 0.1)
  // {
  //   last_time_get = ros::Time::now().toSec();
  //   std::cout << "xHat_ = " << xHat_.transpose() << std::endl;
  //   auto odom_navi = getOdomMsg();
  //   odom_navi.header.stamp = time;
  //   odom_navi.header.frame_id = "odom";
  //   odom_navi.child_frame_id = "base";
  //   odom_navi_pub.publish(odom_navi);
  // }

  return rbdState_;
}

Eigen::Matrix<double, 3, 1> KalmanFilterEstimate::getvel_basefrane()
{
  return getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
}

void KalmanFilterEstimate::updateFromTopic() 
{
  auto* msg = buffer_.readFromRT();

  //! 每次更新 world2sensorTF
  // 收到的是world2sensor
  tf2::Transform world2sensor;
  world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                           msg->pose.pose.orientation.w));

  if (world2odom_.getRotation() == tf2::Quaternion::getIdentity())  // First received
  {
    tf2::Transform odom2sensor;
    try 
    {
      geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("odom", msg->child_frame_id, msg->header.stamp);
      tf2::fromMsg(tf_msg.transform, odom2sensor);
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    world2odom_ = world2sensor * odom2sensor.inverse(); //! 固定的 第一次接受到的时候进行初始化
  }

  tf2::Transform base2sensor; //! 这个应该也是固定的
  try 
  {
    geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("base", msg->child_frame_id, msg->header.stamp);
    tf2::fromMsg(tf_msg.transform, base2sensor);
  } 
  catch (tf2::TransformException& ex) 
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();
  vector3_t newPos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), odom2base.getOrigin().z());

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  vector_t qPino(info_.generalizedCoordinatesNum);
  qPino.head<3>() = newPos;
  qPino.segment<3>(3) = rbdState_.head<3>();
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  pinocchio::forwardKinematics(model, data, qPino);
  pinocchio::updateFramePlacements(model, data);

  //! 主要是更新 xyz 和 feetHeights 的位置
  xHat_.segment<3>(0) = newPos; 
  for (size_t i = 0; i < 4; ++i) 
  {
    xHat_.segment<3>(6 + i * 3) = eeKinematics_->getPosition(vector_t())[i];
    xHat_(6 + i * 3 + 2) -= footRadius_;
    if (contactFlag_[i]) 
    {
      feetHeights_[i] = xHat_(6 + i * 3 + 2);   // 更新feet_height
    }
  }

  auto odom = getOdomMsg();
  odom.header = msg->header;
  odom.child_frame_id = "base";
  publishMsgs(odom);
}


void KalmanFilterEstimate::updateFromOptiTrack()
{
  auto* msg = OptiTrack_buffer_.readFromRT();

  //! 将第一次收到的TF作为offset
  
  if(optiTrack_firstreceived)
  {
    OptiTrack_offset.setOrigin(tf2::Vector3(msg->pose.position.x, msg->pose.position.y, 0));
    OptiTrack_offset.setRotation(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w));
    optiTrack_firstreceived = false;
  }

  OptiTrack_realtime.setOrigin(tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  OptiTrack_realtime.setRotation(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w));

  tf2::Transform OptiTrack_odom = OptiTrack_offset.inverse() * OptiTrack_realtime;
  vector3_t newPos(OptiTrack_odom.getOrigin().x(), OptiTrack_odom.getOrigin().y(), OptiTrack_odom.getOrigin().z());

  geometry_msgs::Vector3 msg1;
  msg1.x = newPos[0];
  msg1.y = newPos[1];
  msg1.z = newPos[2];
  OptiTrack_odom_pub_.publish(msg1);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  vector_t qPino(info_.generalizedCoordinatesNum);
  qPino.head<3>() = newPos;
  qPino.segment<3>(3) = rbdState_.head<3>();
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  pinocchio::forwardKinematics(model, data, qPino);
  pinocchio::updateFramePlacements(model, data);

  xHat_.segment<3>(0) = newPos;

  for(size_t i = 0; i < 4; ++i)
  {
    xHat_.segment<3>(6+i*3) = eeKinematics_->getPosition(vector_t())[i];
    xHat_(6+i*3+2) -= footRadius_;
    if(contactFlag_[i])
    {
      feetHeights_[i] = xHat_(6+i*3+2);
    }
  }

}

void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  buffer_.writeFromNonRT(*msg);
  topicUpdated_ = true;
}

void KalmanFilterEstimate::OptiTrack_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  OptiTrack_buffer_.writeFromNonRT(*msg);
  optiTrackUpdated_ = true;
}


Eigen::Matrix<double, 4, 1> KalmanFilterEstimate::get_localfoot_H()
{
    // std::cout << "foot_height: " << feetHeights_ << std::endl;
  return feetHeights_read;
}

scalar_t KalmanFilterEstimate::get_terrain_angle()
{
  std::cout << "KalmanFilterEstimate::get_terrain_angle not implemented" << std::endl;
  return 0;
}

nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg() {
  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
  odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
  odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
  odom.pose.pose.orientation.x = quat_.x();
  odom.pose.pose.orientation.y = quat_.y();
  odom.pose.pose.orientation.z = quat_.z();
  odom.pose.pose.orientation.w = quat_.w();
  odom.pose.pose.orientation.x = quat_.x();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.pose.covariance[i * 6 + j] = p_(i, j);
      odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
    }
  }
  //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
  vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
  odom.twist.twist.linear.x = twist.x();
  odom.twist.twist.linear.y = twist.y();
  odom.twist.twist.linear.z = twist.z();
  odom.twist.twist.angular.x = angularVelLocal_.x();
  odom.twist.twist.angular.y = angularVelLocal_.y();
  odom.twist.twist.angular.z = angularVelLocal_.z();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
      odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
    }
  }
  return odom;
}

void KalmanFilterEstimate::loadSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "kalmanFilter.";
  if (verbose) {
    std::cerr << "\n #### Kalman Filter Noise:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
}

}  // namespace legged
