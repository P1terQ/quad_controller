#pragma once

#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

namespace legged{

class CerberusVilo : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
public:
    CerberusVilo() = default;
    ~CerberusVilo() override;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& Vilo_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { ViloRunning = false; }

protected:

  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

private:
    std::atomic_bool ViloRunning{};

    ros::Publisher JointState_Pub;
    ros::Publisher IMU_Pub;

    ros::Subscriber JointState_Sub;
    ros::Subscriber IMU_Sub;

    double motorPos[12];
    double motorVel[12];

};


}
