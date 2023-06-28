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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "legged_interface/constraint/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) : config_(std::move(config)), numFeet_(numFeet) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const 
{
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const 
{
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);  //! 找到当前time对应哪个mode_index

  //! trot步态下会 get 3~7的 feetTraj
  // std::cout << "get feetTraj: " << "leg " << leg << " index: " << index << " : " << feetHeightTrajectories_[leg][index].position(time) << std::endl;
  // if(index==7 || index == 3 || index == 2 || index == 8)
  // {
  // std::cout << "get feetTraj: " << "leg " << leg << " index: " << index << std::endl;
  //

  return feetHeightTrajectories_[leg][index].position(time);  //! 返回mode_index对应的轨迹在当前时间下的Z值
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, scalar_t terrainHeight) 
{
  // std::vector<double>
  //  const scalar_array_t terrainHeightSequence1(modeSchedule.modeSequence.size(), 0.0);

  const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);

  feet_array_t<scalar_array_t> liftOffHeightSequence; // std::array< std::vector<scalar_t>, 4>
  liftOffHeightSequence.fill(terrainHeightSequence);

  feet_array_t<scalar_array_t> touchDownHeightSequence;
  touchDownHeightSequence.fill(terrainHeightSequence);
  
  update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, 
                                    const feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                    const feet_array_t<scalar_array_t>& touchDownHeightSequence) 
{
  scalar_array_t heightSequence(modeSchedule.modeSequence.size());
  // std::cout << "modeSchedule.modeSequence.size(): " << modeSchedule.modeSequence.size() << std::endl;  4~10都有可能
  feet_array_t<scalar_array_t> maxHeightSequence;

  for (size_t j = 0; j < numFeet_; j++) 
  {
    for (int p = 0; p < modeSchedule.modeSequence.size(); ++p) 
    {
      heightSequence[p] = std::max(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]);
    }
    maxHeightSequence[j] = heightSequence;  // 4条腿的maxHeightSequence
  }

  update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, 
                                    const feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                    const feet_array_t<scalar_array_t>& touchDownHeightSequence,
                                    const feet_array_t<scalar_array_t>& maxHeightSequence) 
{
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence); //  feet_array_t<std::vector<bool>> 不同mode下对应的contact mode

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;

  for (size_t leg = 0; leg < numFeet_; leg++) 
  {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]); // 针对一条腿的events
  }

  for (size_t j = 0; j < numFeet_; j++) 
  {
    feetHeightTrajectories_[j].clear();
    feetHeightTrajectories_[j].reserve(modeSequence.size());

    for (int p = 0; p < modeSequence.size(); ++p) //? 为啥要遍历每一个mode呢？只生成当前swing feet的trajectory不就好了吗
    {
      if (!eesContactFlagStocks[j][p])  //! check swing mode
      {  
        // for a swing leg
        const int swingStartIndex = startTimesIndices[j][p];
        const int swingFinalIndex = finalTimesIndices[j][p];

        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence); // 在范围内

        const scalar_t swingStartTime = eventTimes[swingStartIndex];
        const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

        const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);  // 1.0

        // 抬腿的 时间， 高度， 速度
        const CubicSpline::Node liftOff{swingStartTime, liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity};

        // 落地的 时间， 高度， 速度
        const CubicSpline::Node touchDown{swingFinalTime, touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity};
        
        // if(liftOffHeightSequence[j][p] !=0 || touchDownHeightSequence[j][p]!=0 )
        // {
        //   std::cout << "legID: " << j << " modeSchdule_index: " << p << std::endl;
        //   std::cout << "liftOffHeightSequence: " << liftOffHeightSequence[j][p] 
        //   << ", touchDownHeightSequence: " << touchDownHeightSequence[j][p] << std::endl;
        // }

        // 中间的最高抬腿高度
        const scalar_t midHeight = maxHeightSequence[j][p] + scaling * config_.swingHeight;

        feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown); // 生成swing trajectory
      } 
      else 
      { // for a stance leg
        // Note: setting the time here arbitrarily to 0.0 -> 1.0 makes the assert in CubicSpline fail
        // const int stanceStartIndex = startTimesIndices[j][p];
        // const int stanceFinalIndex = finalTimesIndices[j][p];
        // checkThatIndicesAreValid(j, p, stanceStartIndex, stanceFinalIndex, modeSequence); 
        // const scalar_t stanceStartTime = eventTimes[stanceStartIndex];
        // const scalar_t stanceFinalTime = eventTimes[stanceFinalIndex];
        // const CubicSpline::Node liftOff{stanceStartTime, liftOffHeightSequence[j][p], 0.0};
        // const CubicSpline::Node touchDown{stanceFinalTime, touchDownHeightSequence[j][p], 0.0};

        const CubicSpline::Node liftOff{0.0, liftOffHeightSequence[j][p], 0.0};
        const CubicSpline::Node touchDown{1.0, touchDownHeightSequence[j][p], 0.0};

        feetHeightTrajectories_[j].emplace_back(liftOff, liftOffHeightSequence[j][p], touchDown);
      }
    }

    feetHeightTrajectoriesEvents_[j] = eventTimes;  // 这个好像没什么用？
  }

  // std::cout<< std::endl << "end of update" << std::endl;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<std::vector<int>, std::vector<int>> SwingTrajectoryPlanner::updateFootSchedule(const std::vector<bool>& contactFlagStock) 
{
  const size_t numPhases = contactFlagStock.size();
  // eg: contactFlagStock: 1 1 0 1 0 1 0 1 0 1 
  // std::cout << "contactFlagStock: ";
  // for (auto i : contactFlagStock)
  // {
  //   std::cout << i << " ";
  // }
  // std::cout << std::endl;

  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing feet
  for (size_t i = 0; i < numPhases; i++) 
  {
    if (!contactFlagStock[i]) // only for swing leg
    {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<bool>> SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const 
{
  const size_t numPhases = phaseIDsStock.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < numFeet_; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock) 
{
  const size_t numPhases = contactFlagStock.size();

  // skip if it is a stance leg
  if (contactFlagStock[index]) 
  {
    std::cout<<"impossible to get here"<<std::endl;
    return {0, 0};
  }

  // find the starting time
  int startTimesIndex = -1;
  for (int ip = index - 1; ip >= 0; ip--) 
  {
    if (contactFlagStock[ip]) 
    {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) 
  {
    if (contactFlagStock[ip]) 
    {
      finalTimesIndex = ip - 1;
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock) {
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0) 
  {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) 
    {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1) 
  {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) 
    {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) 
{
  // std::cout << "(finalTime - startTime) / swingTimeScale: " << (finalTime - startTime) / swingTimeScale << std::endl;
  // (finalTime - startTime) / swingTimeScale都是2.33 所以返回 1,0
  return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### Swing Trajectory Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  SwingTrajectoryPlanner::Config config;
  const std::string prefix = fieldName + ".";

  loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
  loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return config;
}

}  // namespace legged_robot
}  // namespace ocs2
