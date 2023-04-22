/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "legged_interface/SwitchedModelReferenceManager.h"


namespace ocs2 {
namespace legged_robot {

using namespace convex_plane_decomposition;

scalar_t gait_cycle = 0.7;
std::mutex Safety_Mutex;
auto penaltyFunction = [](const Eigen::Vector3d& projectedPoint) { return 0.0; };
scalar_t growthFactor = 1.05;
bool visualize = true;

/******************************************************************************************************/

SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) 
{
  std::cout << "[SwitchedModelReferenceManager Initializer]" << std::endl;
  ros::NodeHandle ReferenceManagerNH;

  for(int i = 0; i < 4; i++)
  {
    FootPlacement_A_Array.at(i).setZero();
    FootPlacement_b_Array.at(i).setZero();
  }
  for(int j = 0; j < 4; j++)
  {
    last_terrainHeight.at(j) = 0.0;
    next_terrainHeight.at(j) = 0.0;
  }

  FootH_now_TerrainF.setZero();

  FootH_now_WorldF.setZero();
  FootH_next_WorldF.setZero();
  FootH_next_next_WorldF.setZero();

  foot_pos_baseF_now.setZero();
  foot_pos_baseF_next.setZero();
  foot_pos_baseF_next_next.setZero();  

  foot_pos_baseF_now_projected.setZero();
  foot_pos_baseF_next_projected.setZero();
  foot_pos_baseF_next_next_projected.setZero();

  Normal_now.setZero();
  Normal_next.setZero();
  Normal_next_next.setZero();

  terrain_angle_ = 0;

  //! A1
                    // LF     RF      LH     RH
  // default_foot_pos_ <<  0.24,   0.24, -0.16, -0.16,
  //                      0.1,  -0.1,    0.08,   -0.08,
  //                     -0.3,  -0.3,   -0.3,   -0.3;

  default_foot_pos_ <<  0.29,   0.29, -0.21, -0.21,
                       0.115,  -0.115,    0.115,   -0.115,
                      -0.37,  -0.37,   -0.37,   -0.37;

  positionPublisher_LF = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("queryPosition_LF", 1);
  projectionPublisher_LF = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_LF", 1);
  // convexTerrainPublisher_LF = ReferenceManagerNH.advertise<geometry_msgs::PolygonStamped>("convex_terrain_LF", 1);
  convexTerrainPublisher_LF = ReferenceManagerNH.advertise<visualization_msgs::Marker>("convex_terrain_LF", 1);

  positionPublisher_RF = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("queryPosition_RF", 1);
  projectionPublisher_RF = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_RF", 1);
  // convexTerrainPublisher_RF = ReferenceManagerNH.advertise<geometry_msgs::PolygonStamped>("convex_terrain_RF", 1);
  convexTerrainPublisher_RF = ReferenceManagerNH.advertise<visualization_msgs::Marker>("convex_terrain_RF", 1);

  positionPublisher_LH = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("queryPosition_LH", 1);
  projectionPublisher_LH = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_LH", 1);
  // convexTerrainPublisher_LH = ReferenceManagerNH.advertise<geometry_msgs::PolygonStamped>("convex_terrain_LH", 1);
  convexTerrainPublisher_LH = ReferenceManagerNH.advertise<visualization_msgs::Marker>("convex_terrain_LH", 1);

  positionPublisher_RH = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("queryPosition_RH", 1);
  projectionPublisher_RH = ReferenceManagerNH.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_RH", 1);
  // convexTerrainPublisher_RH = ReferenceManagerNH.advertise<geometry_msgs::PolygonStamped>("convex_terrain_RH", 1);
  convexTerrainPublisher_RH = ReferenceManagerNH.advertise<visualization_msgs::Marker>("convex_terrain_RH", 1);

  reference_z_offset_publisher = ReferenceManagerNH.advertise<legged_msgs::leggedreference>("/legged_reference_Topic", 1);

  reference_visualization_publisher = ReferenceManagerNH.advertise<visualization_msgs::MarkerArray>("/legged_reference_visualization", 1);
}


void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) 
{
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}


contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const 
{
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}



void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) 
{
  Safety_Mutex.lock();

  // initTime 和 finalTime 相差一秒钟。 取决于task.info 中的 mpc horizion
  // std::cout << "initTime: " << initTime << std::endl;
  // std::cout << "finalTime: " << finalTime << std::endl;

  set_currentTime_userdefined(initTime);

  //! TORSO Reference(z-axis)
  if(planarTerrainPtr_ != nullptr)
  {
    if(targetTrajectories.stateTrajectory[1][0] < 1 && targetTrajectories.stateTrajectory[1][1] < 1 )
    {
      // std::cout << "target vel: " << targetTrajectories.stateTrajectory[1].head(3) << std::endl;  // 现在是robot frame了
      // std::cout << "vel_real: " << targetTrajectories.stateTrajectory[0].segment<3>(0) << std::endl;
      // std::cout << "vel_cmd: " << targetTrajectories.stateTrajectory[1].segment<3>(0) << std::endl; 

      vel_base_real_ = targetTrajectories.stateTrajectory[0].segment<3>(0); //! world_frame下的实际速度
      vel_base_cmd_ = targetTrajectories.stateTrajectory[1].segment<3>(0);  //! world_frame下的当前速度

      base2world_Trans_ = targetTrajectories.stateTrajectory[0].segment<3>(6);  //! 当前状态base->world的translation

      vector3_t tmp_zyx = targetTrajectories.stateTrajectory[0].segment<3>(9);
      base2world_RotMat_ = getRotationMatrixFromZyxEulerAngles(tmp_zyx);  //! 当前状态base->world的rotation
    
      vector3_t base_pos_target_worldF = targetTrajectories.stateTrajectory[1].segment<3>(6);

      grid_map::Position hipLF_target_gm = base_pos_target_worldF.head(2) + default_foot_pos_.col(0).head(2);
      grid_map::Position hipRF_target_gm = base_pos_target_worldF.head(2) + default_foot_pos_.col(1).head(2);
      grid_map::Position hipLH_target_gm = base_pos_target_worldF.head(2) + default_foot_pos_.col(2).head(2);
      grid_map::Position hipRH_target_gm = base_pos_target_worldF.head(2) + default_foot_pos_.col(3).head(2);
      // std::cout << "hipLF_target_gm: " << hipLF_target_gm << std::endl;

      scalar_t z_offset_hipLF = planarTerrainPtr_->gridMap.atPosition("smooth_planar",hipLF_target_gm);
      scalar_t z_offset_hipRF = planarTerrainPtr_->gridMap.atPosition("smooth_planar",hipRF_target_gm);
      scalar_t z_offset_hipLH = planarTerrainPtr_->gridMap.atPosition("smooth_planar",hipLH_target_gm);
      scalar_t z_offset_hipRH = planarTerrainPtr_->gridMap.atPosition("smooth_planar",hipRH_target_gm);

      // grid_map::Position base_target_gm = base_pos_target_worldF.head(2);
      // scalar_t z_offset_base = planarTerrainPtr_->gridMap.atPosition("smooth_planar",base_target_gm);

      legged_msgs::leggedreference reference_msg;
      reference_msg.pitch_offset = terrain_angle_;
      reference_msg.z_offset = (z_offset_hipLF + z_offset_hipRF + z_offset_hipLH + z_offset_hipRH) / 4 * 1.05;
      // reference_msg.z_offset = z_offset_base * 1.5;
      // std::cout << "z_offset: " << reference_msg.z_offset << std::endl;
      reference_z_offset_publisher.publish(reference_msg); 
    }

    // auto layers = planarTerrainPtr_->gridMap.getLayers();
    // for(auto i : layers)
    // {
    //   std::cout << i << std::endl;
    // }
    // smooth
    // normal_x
    // normal_y
    // normal_z
    // plane_classification
    // smooth_before_postprocess
    // elevationWithNaN
    // elevationWithNaN_i
    // elevationWithNaNClosed
    // elevationWithNaNClosedDilated
    // smooth_planar


  }

  const auto timeHorizon = finalTime - initTime;  // MPC 1s

  //! Update Swing Trajectory Reference
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);  //! 0.6 only for 

  scalar_array_t InitTerrainHeightSequence(modeSchedule.modeSequence.size(), 0.0);
  feet_array_t<scalar_array_t> feet_liftOffHeightSequence;  // std::array<T, 4>
  feet_liftOffHeightSequence.fill(InitTerrainHeightSequence);
  feet_array_t<scalar_array_t> feet_touchDownHeightSequence;
  feet_touchDownHeightSequence.fill(InitTerrainHeightSequence);

  //! Update FootPlacement Reference
  const size_t currentMode_index =
    std::lower_bound(modeSchedule.eventTimes.begin(), modeSchedule.eventTimes.end(), initTime) - modeSchedule.eventTimes.begin();
  scalar_t currentMode_timeLeft = modeSchedule.eventTimes[currentMode_index] - initTime;  // 剩余得swing time


  // scalar_t delta_x = 1.05 * std::sqrt(0.45 / 9.8) * (vel_base_real_(0) - vel_base_cmd_(0)) + gait_cycle/2 * vel_base_cmd_(0);
  // scalar_t delta_y = 1.05 * std::sqrt(0.45 / 9.8) * (vel_base_real_(1) - vel_base_cmd_(1)) + gait_cycle/2 * vel_base_cmd_(1);

  //! 就是下降的那一段很奇怪
  if(planarTerrainPtr_ != nullptr) 
  // if(0) 
  {
    for(int i=0; i<modeSchedule.eventTimes.size(); i++)
    {
      if( initTime>modeSchedule.eventTimes.at(i) && initTime<modeSchedule.eventTimes.at(i+1) )
      {
          for(int j = 0; j < 4; j++)
          {
            if(getContactFlags(initTime)[j]) //! 当前Mode(i+1) 是 stance
            {
              // if(planarTerrainPtr_ != nullptr)  //! 在stance的中间时刻, 可以预测下一个mode的swing traj
              {
                scalar_t current_stance_mode_duration = modeSchedule.eventTimes.at(i+1) - modeSchedule.eventTimes.at(i);
                scalar_t next_swing_mode_duration = modeSchedule.eventTimes.at(i+2) - modeSchedule.eventTimes.at(i+1);

                // if(std::abs(currentMode_timeLeft - current_stance_mode_duration/2) < 0.01)
                if(currentMode_timeLeft > current_stance_mode_duration/2)
                {
                  foot_pos_baseF_now.col(j) = base2world_RotMat_ * default_foot_pos_.col(j) + base2world_Trans_;

                  //! 当前foot在world frame的高度
                  foot_pos_baseF_now_projected.col(j) = get_FootHold_Approximation(foot_pos_baseF_now.col(j), false, j);
                  FootH_now_WorldF[j] = foot_pos_baseF_now_projected.col(j)[2];

                  //! 下一步foot在world frame的高度
                  // foot_pos_baseF_next.col(j)[0] = foot_pos_baseF_now.col(j)[0] + delta_x;
                  // foot_pos_baseF_next.col(j)[1] = foot_pos_baseF_now.col(j)[1] + delta_y;
                  foot_pos_baseF_next.col(j)[0] = foot_pos_baseF_now.col(j)[0] + vel_base_cmd_(0) * (currentMode_timeLeft+(next_swing_mode_duration));  //! 目前只用于trot
                  foot_pos_baseF_next.col(j)[1] = foot_pos_baseF_now.col(j)[1] + vel_base_cmd_(1) * (currentMode_timeLeft+(next_swing_mode_duration));
                  foot_pos_baseF_next_projected.col(j) = get_FootHold_Approximation(foot_pos_baseF_next.col(j), true, j);
                  FootH_next_WorldF[j] = foot_pos_baseF_next_projected.col(j)[2];

                  foot_pos_baseF_next_next.col(j)[0] = foot_pos_baseF_next.col(j)[0] + vel_base_cmd_(0) * (currentMode_timeLeft+2*(next_swing_mode_duration));
                  foot_pos_baseF_next_next.col(j)[1] = foot_pos_baseF_next.col(j)[1] + vel_base_cmd_(1) * (currentMode_timeLeft+2*(next_swing_mode_duration));
                  foot_pos_baseF_next_next_projected.col(j) = get_FootHold_Approximation(foot_pos_baseF_next_next.col(j), false, j);
                  FootH_next_next_WorldF[j] = foot_pos_baseF_next_next_projected.col(j)[2];

                  grid_map::Position foot_pos_baseF_now_gm = foot_pos_baseF_now.col(j).head(2);
                  scalar_t normal_x_now = planarTerrainPtr_->gridMap.atPosition("normal_x",foot_pos_baseF_now_gm);
                  scalar_t normal_y_now = planarTerrainPtr_->gridMap.atPosition("normal_y",foot_pos_baseF_now_gm);
                  scalar_t normal_z_now = planarTerrainPtr_->gridMap.atPosition("normal_z",foot_pos_baseF_now_gm);
                  Normal_now.col(j) << normal_x_now, normal_y_now, normal_z_now;

                  grid_map::Position foot_pos_baseF_next_gm = foot_pos_baseF_next.col(j).head(2);
                  scalar_t normal_x_next = planarTerrainPtr_->gridMap.atPosition("normal_x",foot_pos_baseF_next_gm);
                  scalar_t normal_y_next = planarTerrainPtr_->gridMap.atPosition("normal_y",foot_pos_baseF_next_gm);
                  scalar_t normal_z_next = planarTerrainPtr_->gridMap.atPosition("normal_z",foot_pos_baseF_next_gm);
                  Normal_next.col(j) << normal_x_next, normal_y_next, normal_z_next;

                  grid_map::Position foot_pos_baseF_next_next_gm = foot_pos_baseF_next_next.col(j).head(2);
                  scalar_t normal_x_next_next = planarTerrainPtr_->gridMap.atPosition("normal_x",foot_pos_baseF_next_next_gm);
                  scalar_t normal_y_next_next = planarTerrainPtr_->gridMap.atPosition("normal_y",foot_pos_baseF_next_next_gm);
                  scalar_t normal_z_next_next = planarTerrainPtr_->gridMap.atPosition("normal_z",foot_pos_baseF_next_next_gm);
                  Normal_next_next.col(j) << normal_x_next_next, normal_y_next_next, normal_z_next_next;
                }

              }

              //! find next 2 swing mode for leg j
              // feet_liftOffHeightSequence.at(j).at(i) = 0;
              // feet_touchDownHeightSequence.at(j).at(i) = 100;

              for(size_t k = i+2; k<modeSchedule.eventTimes.size(); k++)
              {
                if( !getContactFlags(modeSchedule.eventTimes.at(k))[j] ) // find swing mode for leg j
                {
                  feet_liftOffHeightSequence.at(j).at(k) = FootH_now_WorldF[j];
                  feet_touchDownHeightSequence.at(j).at(k) = FootH_next_WorldF[j];
                  for(size_t l = k+1; l < modeSchedule.eventTimes.size(); l++)
                  {
                    if( !getContactFlags(modeSchedule.eventTimes.at(l))[j] ) 
                    {
                      feet_liftOffHeightSequence.at(j).at(l) = FootH_next_WorldF[j];
                      feet_touchDownHeightSequence.at(j).at(l) = FootH_next_next_WorldF[j];
                      break;
                    }
                  }
                  break;
                }
              }

              // feet_liftOffHeightSequence.at(j).at(i+1) = 0.1;
              // feet_touchDownHeightSequence.at(j).at(i+1) = 0.1;

              // feet_liftOffHeightSequence.at(j).at(i+2) = 0.1; //!
              // feet_touchDownHeightSequence.at(j).at(i+2) = 0.1;

              // feet_liftOffHeightSequence.at(j).at(i+3) = 0.1;
              // feet_touchDownHeightSequence.at(j).at(i+3) = 0.1;

              // feet_liftOffHeightSequence.at(j).at(i+4) = 0.1; //!
              // feet_touchDownHeightSequence.at(j).at(i+4) = 0.1;

            }
            else  //! 当前Mode(i+1) 是 swing
            {
              //! find next 2 swing mode for leg j

              // feet_liftOffHeightSequence.at(j).at(i) = 0;
              // feet_touchDownHeightSequence.at(j).at(i) = 100;

              for(size_t k = i+1; k<modeSchedule.eventTimes.size(); k++)
              {
                if( !getContactFlags(modeSchedule.eventTimes.at(k))[j] ) // find swing mode for leg j
                {
                  feet_liftOffHeightSequence.at(j).at(k) = FootH_now_WorldF[j];
                  feet_touchDownHeightSequence.at(j).at(k) = FootH_next_WorldF[j];
                  for(size_t l = k+1; l < modeSchedule.eventTimes.size(); l++)
                  {
                    if( !getContactFlags(modeSchedule.eventTimes.at(l))[j] ) 
                    {
                      feet_liftOffHeightSequence.at(j).at(l) = FootH_next_WorldF[j];
                      feet_touchDownHeightSequence.at(j).at(l) = FootH_next_next_WorldF[j];
                      break;
                    }
                  }
                  break;
                }
              }

              // feet_liftOffHeightSequence.at(j).at(i+1) = 0.1; //!
              // feet_touchDownHeightSequence.at(j).at(i+1) = 0.1;

              // feet_liftOffHeightSequence.at(j).at(i+2) = 0.1;
              // feet_touchDownHeightSequence.at(j).at(i+2) = 0.1;

              // feet_liftOffHeightSequence.at(j).at(i+3) = 0.1; //!
              // feet_touchDownHeightSequence.at(j).at(i+3) = 0.1;

              // feet_liftOffHeightSequence.at(j).at(i+4) = 0.1; 
              // feet_touchDownHeightSequence.at(j).at(i+4) = 0.1;
            }

          }

        break;
      }

    }
  }
  

  visualize_footholds();

  // std::cout << "FootH_now_WorldF: " << FootH_now_WorldF << std::endl;
  // std::cout << "FootH_next_WorldF: " << FootH_next_WorldF << std::endl;
  // std::cout << "FootH_next_next_WorldF: " << FootH_next_next_WorldF << std::endl;


  // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  // for(int i=0; i<modeSchedule.modeSequence.size(); i++)
  // {
  //   if( i!= modeSchedule.eventTimes.size() && initTime>modeSchedule.eventTimes.at(i) && initTime<modeSchedule.eventTimes.at(i+1) )
  //   {
  //     std::cout << "get i: " << i << std::endl; // i = 3
  //     size_t mode_ =  modeSchedule.modeAtTime(initTime);
  //     std::cout << "mode now: " << mode_ << std::endl;  // mode now = 6
  //     std::cout << "mode " << i << " : " << modeSchedule.modeSequence.at(i) << std::endl; // mode 3 = 9
  //     std::cout << "mode " << i+1 << " : " << modeSchedule.modeSequence.at(i+1) << std::endl; // mode 4 = 6
  //     break;
  //   }
  // }
  // std::cout << "----------------------------------------------------" << std::endl;
  // std::cout << "Time: " << initTime << std::endl; // initTime就是当前时间
  // // std::cout << "finalTime: " << finalTime << std::endl; // finalTime是MPC的time horizon
  // std::cout << "modeSchedule.modeSequence.size: " << modeSchedule.modeSequence.size() << std::endl;
  // std::cout << "modeSchedule.eventTimes.size: " << modeSchedule.eventTimes.size() << std::endl;
  // for(int i=0; i<modeSchedule.modeSequence.size(); i++)
  // {
  //   std::cout << i << "  " << modeSchedule.modeSequence.at(i) << "  ";
  //   if( i == modeSchedule.eventTimes.size() )
  //   {
  //     std::cout << "NAN  ";
  //   }
  //   else
  //   {
  //     std::cout << modeSchedule.eventTimes.at(i)<<"  ";
  //     if( initTime>modeSchedule.eventTimes.at(i) && initTime<modeSchedule.eventTimes.at(i+1))
  //     {
  //       std::cout << " now  ";     
  //     }
  //   }
  //   std::cout << "liftOff: ";
  //   for(int j=0; j<4; j++)
  //   {
  //     std::cout << feet_liftOffHeightSequence.at(j).at(i)<<","; // 第j条腿的第i个mode
  //   }
  //   std::cout << "  ";
  //   std::cout << "touchDown: ";
  //   for(int j=0; j<4; j++)
  //   {
  //     std::cout << feet_touchDownHeightSequence.at(j).at(i)<<",";
  //   }
  //   std::cout << std::endl;
  // }

  //! 这个swingTraj 是local terrain frame吗？
  swingTrajectoryPtr_->update(modeSchedule, feet_liftOffHeightSequence, feet_touchDownHeightSequence);

  // swingTrajectoryPtr_->update(modeSchedule, 0); //! 是liftoff永远为0，touchdown是相对于Liftoff吗？ 落足点高度

  Safety_Mutex.unlock();
}

void SwitchedModelReferenceManager::update(std::unique_ptr<convex_plane_decomposition::PlanarTerrain>& New_TerrainMap_Ptr, scalar_t terrain_angle)
{
  Safety_Mutex.lock();  //! update和read不能同时进行

  planarTerrainPtr_.swap(New_TerrainMap_Ptr);
  terrain_angle_ = terrain_angle;

  Safety_Mutex.unlock();
}

geometry_msgs::PointStamped toMarker(const Eigen::Vector3d& position, const std_msgs::Header& header) {
  geometry_msgs::PointStamped sphere;
  
  sphere.header = header;
  sphere.point.x = position.x();
  sphere.point.y = position.y();
  sphere.point.z = position.z();
  return sphere;
}

vector3_t SwitchedModelReferenceManager::get_FootHold_Approximation(vector3_t FootHold_init, bool ifNeed_Ab, size_t FootId)
{
  // auto penaltyFunction = [](const Eigen::Vector3d& projectedPoint) { return 0.0; };

  PlanarTerrainProjection FootHold_projection = getBestPlanarRegionAtPositionInWorld(FootHold_init, planarTerrainPtr_->planarRegions, penaltyFunction);

  if(ifNeed_Ab)
  {
    //! CgalPolygon2d
    const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
      FootHold_projection.regionPtr->boundaryWithInset.boundary, FootHold_projection.positionInTerrainFrame, NumVertex, growthFactor);

    std_msgs::Header header;
    header.frame_id = "odom";

    //! geometry_msgs::PolygonStamped
    auto convexRegionMsg =  
        convex_plane_decomposition::to3dRosPolygon(convexRegion, FootHold_projection.regionPtr->transformPlaneToWorld, header); //! 这边得到的确实是 world frame

    //! visualization_msgs::Marker
    auto convexRegionMsg_colored =
        convex_plane_decomposition::to3dRosMarker_(convexRegion,
                                                  FootHold_projection.regionPtr->transformPlaneToWorld,
                                                  header,
                                                  convex_plane_decomposition::getColor_(FootId, 1.0),
                                                  FootId,
                                                  0.01); // lineWidth


    if(visualize)
    {

      switch (FootId)
      {
      case 0:
      {
        // convexTerrainPublisher_LF.publish(convexRegionMsg);
        convexTerrainPublisher_LF.publish(convexRegionMsg_colored);
        // projectionPublisher_LF.publish(toMarker(FootHold_projection.positionInWorld, header));
      }
      break;
      
      case 1:
      {
        // convexTerrainPublisher_RF.publish(convexRegionMsg);
        convexTerrainPublisher_RF.publish(convexRegionMsg_colored);
        // projectionPublisher_RF.publish(toMarker(FootHold_projection.positionInWorld, header));
      }
        break;

      case 2:
      {
        // convexTerrainPublisher_LH.publish(convexRegionMsg);
        convexTerrainPublisher_LH.publish(convexRegionMsg_colored);
        // projectionPublisher_LH.publish(toMarker(FootHold_projection.positionInWorld, header));
      }
        break;

      case 3:
      {
        // convexTerrainPublisher_RH.publish(convexRegionMsg);
        convexTerrainPublisher_RH.publish(convexRegionMsg_colored);
        // projectionPublisher_RH.publish(toMarker(FootHold_projection.positionInWorld, header));
      }
        break;

      default:
      {
        std::cout << "wrong visualization ID" << std::endl;
      }
        break;
      }

    }


    for(int i=0; i<NumVertex-1; i++)
    {
      double A = (convexRegionMsg.polygon.points[i+1].y - convexRegionMsg.polygon.points[i].y)/(convexRegionMsg.polygon.points[i+1].x - convexRegionMsg.polygon.points[i].x);
      double Coeff_y = -1.0;
      double b = (convexRegionMsg.polygon.points[i].y * convexRegionMsg.polygon.points[i+1].x - convexRegionMsg.polygon.points[i+1].y * convexRegionMsg.polygon.points[i].x) / (convexRegionMsg.polygon.points[i+1].x - convexRegionMsg.polygon.points[i].x);

      if(A * FootHold_projection.positionInWorld(0) - FootHold_projection.positionInWorld(1) + b >= 0)
      {
        FootPlacement_A_Array.at(FootId)(i,0) = A;
        FootPlacement_A_Array.at(FootId)(i,1) = -1.0;
        FootPlacement_A_Array.at(FootId)(i,2) = 0.0;
        FootPlacement_b_Array.at(FootId)(i,0) = b;
      }
      else
      {
        FootPlacement_A_Array.at(FootId)(i,0) = -A;
        FootPlacement_A_Array.at(FootId)(i,1) = 1.0;
        FootPlacement_A_Array.at(FootId)(i,2) = 0.0;
        FootPlacement_b_Array.at(FootId)(i,0) = -b;
      }
    }
    double A = (convexRegionMsg.polygon.points[NumVertex-1].y - convexRegionMsg.polygon.points[0].y)/(convexRegionMsg.polygon.points[NumVertex-1].x - convexRegionMsg.polygon.points[0].x);
    double Coeff_y = -1.0;
    double b = (convexRegionMsg.polygon.points[0].y * convexRegionMsg.polygon.points[NumVertex-1].x - convexRegionMsg.polygon.points[NumVertex-1].y * convexRegionMsg.polygon.points[0].x) / (convexRegionMsg.polygon.points[NumVertex-1].x - convexRegionMsg.polygon.points[0].x);

    if(A * FootHold_projection.positionInWorld(0) - FootHold_projection.positionInWorld(1) + b >= 0)
    {
        FootPlacement_A_Array.at(FootId)(NumVertex-1,0) = A;
        FootPlacement_A_Array.at(FootId)(NumVertex-1,1) = -1.0;
        FootPlacement_A_Array.at(FootId)(NumVertex-1,2) = 0.0;
        FootPlacement_b_Array.at(FootId)(NumVertex-1,0) = b;
    }
    else
    {
      FootPlacement_A_Array.at(FootId)(NumVertex-1,0) = -A;
      FootPlacement_A_Array.at(FootId)(NumVertex-1,1) = 1.0;
      FootPlacement_A_Array.at(FootId)(NumVertex-1,2) = 0.0;
      FootPlacement_b_Array.at(FootId)(NumVertex-1,0) = -b;
    }  

  }

  return FootHold_projection.positionInWorld; 
}

void SwitchedModelReferenceManager::visualize_footholds()
{
  visualization_msgs::MarkerArray Foothold_visulization_msg;
  Foothold_visulization_msg.markers.reserve(12); // 4 * 3

  vector3_t n_direction_now = {0.0, 0.0, 0.12};
  vector3_t n_direction_next = {0.0, 0.0, 0.105};  //todo 目前没有估计坡面的斜率，都假设是竖直方向的
  vector3_t n_direction_next_next = {0.0, 0.0, 0.09}; 

  //! 再调以下alpha
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_now.col(0)*0.12, foot_pos_baseF_now_projected.col(0), Color::blue));
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next.col(0)*0.105, foot_pos_baseF_next_projected.col(0), Color::blue));
  // Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next_next.col(0)*0.09, foot_pos_baseF_next_next_projected.col(0), Color::blue));
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_now.col(1)*0.12, foot_pos_baseF_now_projected.col(1), Color::orange));
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next.col(1)*0.105, foot_pos_baseF_next_projected.col(1), Color::orange));
  // Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next_next.col(1)*0.09, foot_pos_baseF_next_next_projected.col(1), Color::orange));
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_now.col(2)*0.12, foot_pos_baseF_now_projected.col(2), Color::yellow));
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next.col(2)*0.105, foot_pos_baseF_next_projected.col(2), Color::yellow));
  // Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next_next.col(2)*0.09, foot_pos_baseF_next_next_projected.col(2), Color::yellow));
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_now.col(3)*0.12, foot_pos_baseF_now_projected.col(3), Color::purple));
  Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next.col(3)*0.105, foot_pos_baseF_next_projected.col(3), Color::purple));
  // Foothold_visulization_msg.markers.emplace_back(getArrowAtPointMsg(Normal_next_next.col(3)*0.09, foot_pos_baseF_next_next_projected.col(3), Color::purple));

  assignHeader(Foothold_visulization_msg.markers.begin(), Foothold_visulization_msg.markers.end(), getHeaderMsg("odom", ros::Time::now()));
  assignIncreasingId(Foothold_visulization_msg.markers.begin(), Foothold_visulization_msg.markers.end());

  reference_visualization_publisher.publish(Foothold_visulization_msg);
}

}  // namespace legged_robot
}  // namespace ocs2
