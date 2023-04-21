//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
}  // namespace

//! linear estimate time needed from now to target
scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) 
{
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, 
                                                  const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) 
{
  // desired time trajectory
  //! time traj 由 当前时间和目标时间组成
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};  // targetReachingTime = observation.time + mpc.horizon

  // desired state trajectory
  //! state traj: state_now, state_target
  vector_t currentPose = observation.state.segment<6>(6);

  // std::cout << "observation.state.tail(12)" << observation.state.tail(12) << std::endl;

  // currentPose(2) = COM_HEIGHT;  //? 
  // std::cout << "currentPose(2)" << currentPose(2) << std::endl; // 实际的state_estimation的高度
  // currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  
  //! 那这边这个currentPose有啥用
  stateTrajectory[0] << vector_t::Zero(6), currentPose, observation.state.tail(12);
  // stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  //! input traj: both zero
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

//! 这个是rviz里边的，不管
TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) 
{

  const vector_t currentPose = observation.state.segment<6>(6);

  const vector_t targetPose = [&]() 
  {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT; //! this shouldn't be a const
    target(3) = goal(3);
    target(4) = 0;  
    target(5) = 0;
    return target;
  }();

  //! linear estimate time needed to get target
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);

  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) 
{
  const vector_t currentPose = observation.state.segment<6>(6);

  //! transform cmdvel from world frame to robot_ori-aligned frame
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  const scalar_t timeToTarget = TIME_TO_TARGET; // mpc.horizon = 1s

  //! linear estimate target pose
  const vector_t targetPose = [&]() 
  {
    vector_t target(6);

    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget; //! x_goal = x_now + v * delta_T
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget; //! y_goal = y_now + v * delta_T

    // target(2) = COM_HEIGHT + z_offset;
    target(2) = COM_HEIGHT + 0.05 + z_offset;

    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;  //! yaw_goal = yaw_now + v * delta_T
    //TODO: 加上yaw和pitch上的控制
    target(4) = terrain_pitch;  
    // target(4) = 0; 
    target(5) = 0;

    // std::cout << "target_pose: " << target << std::endl;
    
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;

  //! transform target pose to trajectory
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  
  //! 还是用world_frame下的速度，按照observation来
  trajectories.stateTrajectory[0].head(3) = observation.state.segment<3>(0);    // state_now_vel
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;   // state_target_vel

  z_offset_last = z_offset;

  // std::cout << "stateTrajectory.size()" << trajectories.stateTrajectory.size() << std::endl;

  std::cout << "currentVel: " << trajectories.stateTrajectory[0].segment<6>(0) << std::endl;
  std::cout << "targetVel: " << trajectories.stateTrajectory[1].segment<6>(0) << std::endl;
  std::cout << "currentPose: " << trajectories.stateTrajectory[0].segment<6>(6) << std::endl;
  std::cout << "targetPose: " << trajectories.stateTrajectory[1].segment<6>(6) << std::endl;

  return trajectories;
}

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  z_offset = 0.0;
  z_offset_last = 0.0;
  terrain_pitch = 0.0;

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);

  ros::spin();  

  // Successful exit
  return 0;
}
