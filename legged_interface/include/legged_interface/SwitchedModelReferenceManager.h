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

#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <ocs2_mpc/SystemObservation.h>

#include "ocs2_robotic_tools/common/RotationTransforms.h"

#include "legged_interface/constraint/SwingTrajectoryPlanner.h"

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <legged_msgs/leggedreference.h>

#include <visualization_msgs/Marker.h>

namespace ocs2 {
namespace legged_robot {

#define NumVertex 8
using namespace convex_plane_decomposition;

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~SwitchedModelReferenceManager() override = default;

  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

  void set_currentTime_userdefined(scalar_t time) {currentTime_userdefined = time;}

  scalar_t get_currentTime_userdefined() const {return currentTime_userdefined;}

  Eigen::Matrix<scalar_t, NumVertex, 3> get_FootPlacement_A(const size_t contactPointIndex) const {return FootPlacement_A_Array.at(contactPointIndex);}
  Eigen::Matrix<scalar_t, NumVertex, 1> get_FootPlacement_b(const size_t contactPointIndex) const {return FootPlacement_b_Array.at(contactPointIndex);}

  void update(std::unique_ptr<convex_plane_decomposition::PlanarTerrain>& New_TerrainMap_Ptr, scalar_t terrain_angle);

  vector3_t get_FootHold_Approximation(vector3_t FootHold_init, bool ifNeed_Ab, size_t FootId);

  vector3_t getSurfaceNormal(const size_t footid) const { return Normal_now.col(footid); }

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;

  scalar_t currentTime_userdefined;

  feet_array_t<Eigen::Matrix<scalar_t, NumVertex, 3>> FootPlacement_A_Array;
  feet_array_t<Eigen::Matrix<scalar_t, NumVertex, 1>> FootPlacement_b_Array;

  feet_array_t<scalar_t> last_terrainHeight;
  feet_array_t<scalar_t> next_terrainHeight;

  vector3_t vel_base_real_;
  vector3_t vel_base_cmd_;
  vector3_t base2world_Trans_;
  matrix3_t base2world_RotMat_;
  std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  Eigen::Matrix<scalar_t, 4, 1> FootH_now_TerrainF;

  Eigen::Matrix<scalar_t, 4, 1> FootH_now_WorldF;
  Eigen::Matrix<scalar_t, 4, 1> FootH_next_WorldF;
  Eigen::Matrix<scalar_t, 4, 1> FootH_next_next_WorldF;

  Eigen::Matrix<scalar_t, 3, 4> default_foot_pos_;

  Eigen::Matrix<scalar_t, 3, 4> foot_pos_baseF_now; // 当前(默认)foot在世界系下的位置
  Eigen::Matrix<scalar_t, 3, 4> foot_pos_baseF_next;  // 下一个mode(默认)foot在世界系下的位置
  Eigen::Matrix<scalar_t, 3, 4> foot_pos_baseF_next_next; // 下下个mode(默认)foot在世界系下的位置
  Eigen::Matrix<scalar_t, 3, 4> foot_pos_baseF_now_projected; // 当前(投影后)foot在世界系下的位置
  Eigen::Matrix<scalar_t, 3, 4> foot_pos_baseF_next_projected;  // 下一个mode(投影后)foot在世界系下的位置
  Eigen::Matrix<scalar_t, 3, 4> foot_pos_baseF_next_next_projected; // 下下个mode(投影后)foot在世界系下的位置

  Eigen::Matrix<scalar_t, 3, 4> Normal_now;
  Eigen::Matrix<scalar_t, 3, 4> Normal_next;
  Eigen::Matrix<scalar_t, 3, 4> Normal_next_next;

  scalar_t terrain_angle_;

  ros::Publisher positionPublisher_LF;
  ros::Publisher projectionPublisher_LF;
  ros::Publisher convexTerrainPublisher_LF;
  ros::Publisher positionPublisher_RF;
  ros::Publisher projectionPublisher_RF;
  ros::Publisher convexTerrainPublisher_RF;
  ros::Publisher positionPublisher_LH;
  ros::Publisher projectionPublisher_LH;
  ros::Publisher convexTerrainPublisher_LH;
  ros::Publisher positionPublisher_RH;
  ros::Publisher projectionPublisher_RH;
  ros::Publisher convexTerrainPublisher_RH;

  ros::Publisher reference_z_offset_publisher;

  ros::Publisher reference_visualization_publisher;

  // visualization_msgs::MarkerArray Foothold_visulization;

  void visualize_footholds(void);


};

}  // namespace legged_robot
}  // namespace ocs2
