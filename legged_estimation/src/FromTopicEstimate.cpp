//
// Created by qiayuan on 2022/7/24.
//
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/FromTopiceEstimate.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {

  double last_time_get_2;


// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}
double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}

FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) 
{
  ros::NodeHandle nh;
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);  // cheatEstimator 订阅的是 gazebo发布的/ground_truth/state话题

  eeKinematics_->setPinocchioInterface(pinocchioInterface_);
  z_first_offset = 0;
  first_received = true;
  feetHeights_read.setZero();

  terrain_angle = 0;
  terrain_angle_filter = MovingWindowFilter(100);
  for(size_t i = 0; i < 4; ++i)
  {
    recent_contact_x_filter[i] = MovingWindowFilter(60);
    recent_contact_y_filter[i] = MovingWindowFilter(60);
    recent_contact_z_filter[i] = MovingWindowFilter(60);
  }

  last_time_get_2 = ros::Time::now().toSec();

  odom_navi_pub = nh.advertise<nav_msgs::Odometry>("/odom_navi", 1);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const ros::Time& time, const ros::Duration& /*period*/) {
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  if(first_received)
  {
    z_first_offset = odom.pose.pose.position.z;
    std::cout << "z_first_offset: " << z_first_offset << std::endl;
    first_received = false;
  }
  
  updateAngular(quatToZyx(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
                Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
  updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z-z_first_offset),
               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  vel_baseframe = getRotationMatrixFromZyxEulerAngles(quatToZyx(Eigen::Quaternion<scalar_t>
    (odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z))).transpose() *
      Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);


  //! Kinematic Computation
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();  
  
  vector_t qPino(info_.generalizedCoordinatesNum);
  vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(0) = rbdState_.segment<3>(3);  //  XYZ Pos
  qPino.segment<3>(3) = rbdState_.head<3>();  // ZYX Ori
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);  // Dof Pos
  // std::cout << qPino.size() << std::endl; // 18
  vPino.setZero();
  vPino.segment<3>(0) = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3),
      rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  vPino.tail(info_.actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);
  const auto eePos = eeKinematics_->getPosition(vector_t());
  // std::cout << "eePos[0][2]: " << eePos[0][2] << std::endl;
  


  { //! compute walking surface
    for(int i=0; i<4; ++i)
    {
      if(contactFlag_[i])
      {
        foot_pos_recent_contact.block<3,1>(0,i) << recent_contact_x_filter[i].CalculateAverage(eePos[i][0]),
                                                  recent_contact_y_filter[i].CalculateAverage(eePos[i][1]),
                                                  recent_contact_z_filter[i].CalculateAverage(eePos[i][2]);
      }
    }
    Eigen::Matrix<scalar_t, 4, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d surf_coef;
    Eigen::Vector3d flat_ground_coef;

    W.block<4,1>(0,0).setOnes();
    W.block<4,2>(0,1) = foot_pos_recent_contact.block<2,4>(0,0).transpose();

    foot_pos_z.resize(4);
    foot_pos_z = foot_pos_recent_contact.block<1,4>(2,0).transpose();

    a = pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    flat_ground_coef << 0, 0, 1;

    if(rbdState_(5)>0.6)
    {
      terrain_angle = terrain_angle_filter.CalculateAverage(cal_dihedral_angle(flat_ground_coef, surf_coef));
    }
    else
    {
      terrain_angle = 0;
    }
    if (terrain_angle > 0.5) {
        terrain_angle = 0.5;
    }
    if (terrain_angle < -0.5) {
        terrain_angle = -0.5;
    }
  }

  odom.header.stamp = time;
  publishMsgs(odom);

  // last_time_get_2 = ros::Time::now().toSec();
  // if(ros::Time::now().toSec() - last_time_get_2 > 0.5)
  // {
  //   last_time_get_2 = ros::Time::now().toSec();
  //   // std::cout << "xHat_ = " << xHat_.transpose() << std::endl;
  //   // auto odom_navi = getOdomMsg();
  //   // odom_navi.header.stamp = time;
  //   // odom_navi.header.frame_id = "odom";
  //   // odom_navi.child_frame_id = "base";
  //   // nav_msgs::Odometry odom_navi = odom.copy();
  //   nav_msgs::Odometry odom_navi = *buffer_.readFromRT();
  //   odom_navi.header.frame_id = "odom_navi";
  //   // odom_navi.
  //   std::cout << odom_navi.header.frame_id << std::endl;
  //   std::cout << odom_navi.child_frame_id << std::endl;
  //   std::cout << "publish odom_navi" << std::endl;
  //   odom_navi_pub.publish(odom_navi);
  // }
  // else
  // {
  //   std::cout << "ros::Time::now().toSec(): " << ros::Time::now().toSec()  << std::endl;
  //   std::cout << "last_time_get_2: " << last_time_get_2  << std::endl;
  //   std::cout << "odom_navi not publish: " << ros::Time::now().toSec() - last_time_get_2  << std::endl;
  // }

  return rbdState_;
}

Eigen::Matrix<double, 3, 1> FromTopicStateEstimate::getvel_basefrane()
{
  return vel_baseframe;
}

Eigen::Matrix<double, 4, 1> FromTopicStateEstimate::get_localfoot_H()
{
  return feetHeights_read;
} 

scalar_t FromTopicStateEstimate::get_terrain_angle()
{
  scalar_t Front_Hind_diffH = foot_pos_recent_contact(2,0) + foot_pos_recent_contact(2,1)
   - foot_pos_recent_contact(2,2) - foot_pos_recent_contact(2,3);

  if(Front_Hind_diffH>0.05)
  {
    return -terrain_angle;
  } 
  else
  {
    return terrain_angle;
  }

}


}  // namespace legged
