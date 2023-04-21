#include "cerbrus_interface/CerberusVilo.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged{

bool CerberusVilo::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& Vilo_nh) 
{
    std::cout << "CerberusVilo Init" << std::endl;

    JointState_Pub = Vilo_nh.advertise<sensor_msgs::JointState>("/hardware_a1/joint_foot", 1);
    IMU_Pub = Vilo_nh.advertise<sensor_msgs::Imu>("/hardware_a1/imu", 1);

    // auto JointStateCallback = [this](const sensor_msgs::JointState::ConstPtr& msg)
    // {
    //     msg->position.at[0]
    // };


    // JointState_Sub = Vilo_nh.subscribe<sensor_msgs::JointState>("/trunk_imu",1 )

    return true;
}

void CerberusVilo::starting(const ros::Time& time)
{
    std::cout << "CerberusVilo Start" << std::endl;

    sensor_msgs::JointState JointState_MSG;

    // JointState_MSG.header.frame_id
    // JointState_MSG.header.seq
    JointState_MSG.header.stamp = time;
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                        "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
    JointState_MSG.name = joint_names;
    for(size_t i = 0; i < hybridJointHandles_.size(); ++i)
    {
        JointState_MSG.position.push_back(hybridJointHandles_[i].getPosition());
        JointState_MSG.velocity.push_back(hybridJointHandles_[i].getVelocity());
    }

    sensor_msgs::Imu IMU_MSG;
    IMU_MSG.header.stamp = time;
    IMU_MSG.angular_velocity.x = imuSensorHandle_.getAngularVelocity()[0];
    IMU_MSG.angular_velocity.y = imuSensorHandle_.getAngularVelocity()[1];
    IMU_MSG.angular_velocity.z = imuSensorHandle_.getAngularVelocity()[2];
    IMU_MSG.linear_acceleration.x = imuSensorHandle_.getLinearAcceleration()[0];
    IMU_MSG.linear_acceleration.y = imuSensorHandle_.getLinearAcceleration()[1];
    IMU_MSG.linear_acceleration.z = imuSensorHandle_.getLinearAcceleration()[2];

    JointState_Pub.publish(JointState_MSG);
    IMU_Pub.publish(IMU_MSG);
    ViloRunning = true;

}

void CerberusVilo::update(const ros::Time& time, const ros::Duration& period)
{
    if(ViloRunning)
    {
        std::cout << "CerberusVilo Update" << std::endl;
        sensor_msgs::JointState JointState_MSG;
        JointState_MSG.header.stamp = time;
        std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                            "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
        JointState_MSG.name = joint_names;
        for(size_t i = 0; i < hybridJointHandles_.size(); ++i)
        {
            JointState_MSG.position.push_back(hybridJointHandles_[i].getPosition());
            JointState_MSG.velocity.push_back(hybridJointHandles_[i].getVelocity());
        }

        sensor_msgs::Imu IMU_MSG;
        IMU_MSG.header.stamp = time;
        IMU_MSG.angular_velocity.x = imuSensorHandle_.getAngularVelocity()[0];
        IMU_MSG.angular_velocity.y = imuSensorHandle_.getAngularVelocity()[1];
        IMU_MSG.angular_velocity.z = imuSensorHandle_.getAngularVelocity()[2];
        IMU_MSG.linear_acceleration.x = imuSensorHandle_.getLinearAcceleration()[0];
        IMU_MSG.linear_acceleration.y = imuSensorHandle_.getLinearAcceleration()[1];
        IMU_MSG.linear_acceleration.z = imuSensorHandle_.getLinearAcceleration()[2];

        JointState_Pub.publish(JointState_MSG);
        IMU_Pub.publish(IMU_MSG);        
    }

}

CerberusVilo::~CerberusVilo()
{
    ViloRunning = false;
    std::cout << "CerberusVilo Stop" << std::endl;
}


}

PLUGINLIB_EXPORT_CLASS(legged::CerberusVilo, controller_interface::ControllerBase)


