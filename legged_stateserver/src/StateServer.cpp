#include "legged_stateserver/StateServer.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged{

bool StateServer::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) 
{
    std::cout << "StateServer Init" << std::endl;

    Joint_Publisher = nh.advertise<sensor_msgs::JointState>("/legged_state/joint", 1);
    IMU_Publisher = nh.advertise<sensor_msgs::Imu>("/legged_state/imu", 1);
    LowState_Publisher = nh.advertise<legged_msgs::lowstate>("/legged_state/lowstate", 1);

    auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
    for (const auto& joint_name : joint_names) {
        jointStateHandles_.push_back(robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(joint_name));
    }
    auto* contactInterface = robot_hw->get<ContactSensorInterface>();

    std::vector<std::string> contactNames3DoF{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    for (const auto& name : contactNames3DoF) {
        contactHandles_.push_back(contactInterface->getHandle(name));
    }
    imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");


    return true;
}

void StateServer::starting(const ros::Time& time)
{
    std::cout << "StateServer Start" << std::endl;

    time_ = ros::Time::now().toSec();

    StateServerRunning = true;
}

void StateServer::update(const ros::Time& time, const ros::Duration& period)
{   
    if(StateServerRunning && (ros::Time::now().toSec() - time_)>0.0009)
    {
        for (size_t i = 0; i < 12; ++i) 
        {
            motorPos(i) = jointStateHandles_[i].getPosition();
            motorVel(i) = jointStateHandles_[i].getVelocity();
        }
        for (size_t i = 0; i < 4; ++i) {
            FootcontactFlag[i] = contactHandles_[i].isContact();
        }
        for (size_t i = 0; i < 4; ++i) {
            IMUquat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
        }
        for (size_t i = 0; i < 3; ++i) {
            IMUangularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
            IMUlinearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
        }

        legged_msgs::lowstate lowstate_msg;
        for(size_t i = 0; i < 12; ++i)
        {
            lowstate_msg.jointPos[i] = motorPos(i);
            lowstate_msg.jointVel[i] = motorVel(i);
            // std::cout << "lowstate_msg.jointPos" << i <<" : " << lowstate_msg.jointPos[i] << std::endl;
            // std::cout << "lowstate_msg.jointVel" << i <<" : " << lowstate_msg.jointVel[i] << std::endl;
        }
        for(size_t i = 0; i < 4; ++i)
        {
            lowstate_msg.contactFlag[i] = FootcontactFlag[i];
            // std::cout << "FootcontactFlag" << i << " : " << FootcontactFlag[i] << std::endl;
            // std::cout << "lowstate_msg.contactFlag" << i <<" : " << lowstate_msg.contactFlag[i] << std::endl;
        }
        for(size_t i = 0; i < 4; ++i)
        {
            lowstate_msg.IMUQuat[i] = IMUquat.coeffs()(i);
            // std::cout << "lowstate_msg.IMUQuat" << i <<" : " << lowstate_msg.IMUQuat[i] << std::endl;
        }
        for(size_t i = 0; i < 3; ++i)
        {
            lowstate_msg.IMUangularVel[i] = IMUangularVel(i);
            lowstate_msg.IMUlinearAccel[i] = IMUlinearAccel(i);
            // std::cout << "lowstate_msg.IMUangularVel" << i <<" : " << lowstate_msg.IMUangularVel[i] << std::endl;
            // std::cout << "lowstate_msg.IMUlinearAccel" << i <<" : " << lowstate_msg.IMUlinearAccel[i] << std::endl;
        }
        LowState_Publisher.publish(lowstate_msg);

        sensor_msgs::Imu IMU_msg;
        IMU_msg.header.stamp = time;
        IMU_msg.angular_velocity.x = IMUangularVel(0);
        IMU_msg.angular_velocity.y = IMUangularVel(1);
        IMU_msg.angular_velocity.z = IMUangularVel(2);
        IMU_msg.linear_acceleration.x = IMUlinearAccel(0);
        IMU_msg.linear_acceleration.y = IMUlinearAccel(1);
        IMU_msg.linear_acceleration.z = IMUlinearAccel(2);
        IMU_msg.orientation.x = IMUquat.coeffs()(0);
        IMU_msg.orientation.y = IMUquat.coeffs()(1);
        IMU_msg.orientation.z = IMUquat.coeffs()(2);
        IMU_msg.orientation.w = IMUquat.coeffs()(3);
        IMU_Publisher.publish(IMU_msg);

        sensor_msgs::JointState Joint_msg;
        Joint_msg.header.stamp = time;

        for(size_t i = 0; i < 12; ++i)
        {
            Joint_msg.name.push_back(jointStateHandles_[i].getName());
            Joint_msg.position.push_back(motorPos(i));
            Joint_msg.velocity.push_back(motorVel(i));
            Joint_msg.effort.push_back(jointStateHandles_[i].getEffort());
        }

        Joint_Publisher.publish(Joint_msg);

        time_ = ros::Time::now().toSec();
        // std::cout << time_ << std::endl;
    }

}

StateServer::~StateServer()
{
    StateServerRunning = false;
    std::cout << "StateServer Stop" << std::endl;
}


}

PLUGINLIB_EXPORT_CLASS(legged::StateServer, controller_interface::ControllerBase)


