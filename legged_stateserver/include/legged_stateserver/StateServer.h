#pragma once

#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include "legged_msgs/lowstate.h"
#include <Eigen/Dense>
#include <ros/time.h>

namespace legged{

class StateServer : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
public:
    StateServer() = default;
    ~StateServer() override;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { StateServerRunning = false; }

protected:

  ros::Publisher IMU_Publisher;
  ros::Publisher Joint_Publisher;
  ros::Publisher LowState_Publisher;

  std::vector<hardware_interface::JointStateHandle> jointStateHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

private:
    std::atomic_bool StateServerRunning{};

    Eigen::Matrix<double, 12, 1> motorPos;
    Eigen::Matrix<double, 12, 1> motorVel;
    Eigen::Matrix<bool, 4, 1> FootcontactFlag;
    Eigen::Quaternion<double> IMUquat;
    Eigen::Matrix<double, 3, 1> IMUangularVel, IMUlinearAccel;

    double time_;

};


}
